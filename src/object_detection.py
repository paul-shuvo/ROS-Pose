#!/usr/bin/env python
# -*- coding: future_fstrings -*-

from os.path import dirname, abspath, join
from os import listdir, chdir
import numpy as np
from time import time
import cv2
import json
# ROS imports
import rospy
from std_msgs.msg import String
import config
from utils import shrink_bbox, draw_angled_text

# Change directory to the file directory
dir_path = dirname(abspath(__file__))
chdir(dir_path)

# TODO: check hold functionality for multiple objects


class ObjectDetection():
    def __init__(self, config, to_gray: bool = False):
        """
        Initializes the object detection class

        Parameters
        ----------
        config : object
            Contains the configuration for the class.
        to_gray : bool, optional
            If true converts images to gray else keeps
            them in original format, by default False
        """
        self.config = config
        self.to_gray = config.to_gray
        self.detector_descriptor = config.detector_descriptor
        self.matcher = config.matcher

        if self.config.show_image:
            self.viz_frame = None

        # Holds all the information i.e. keypoints,
        # descriptors, and dimension, for object detection.
        self.query_object_features = {}

        if self.config.hold_prev_vals:
            self.object_timer = {}

        # Retrieves all the image feature info needed
        # for each of the object for detection
        self.extract_object_info(self.config.objects, self.config.object_path)

        # Set frame rate
        self.frame_rate = 2
        self.prev = 0

        # Initializes ros node for object detection
        rospy.init_node('object_detection', anonymous=False)
        # `self.obj_boundary_info` is converted to json string and
        # then set to `self.obj_boundary_msg` which then gets published
        self.obj_boundary_msg = ''
        self.obj_boundary_info = {}
        self.obj_boundary_pub = rospy.Publisher(
                                    '/detected_object',
                                    String,
                                    queue_size=10
                                )

        # Starts the subscriber
        r = rospy.Rate(self.frame_rate)
        while not rospy.is_shutdown():
            rospy.Subscriber(
                    self.config.image_sub['topic'],
                    self.config.image_sub['type'],
                    self.callback
                )
            r.sleep()

    def extract_object_info(self, objects, obj_path: str):
        """
        Extracts all the information and features needed for
        object detection.

        Parameters
        ----------
        objects : list | str
            List of object names.
        obj_path : str
            Path to the folder containing object images.
        """
        # Image extensions that are supported
        supported_formats = ['jpg', 'jpeg', 'png']

        if objects == 'all':
            image_files = [join(obj_path, f) for f in listdir(obj_path)
                           if f.endswith(('.jpg', '.png'))]
        else:
            image_files = []
            for f in listdir(obj_path):
                obj, ext = f.split('.')
                for object_ in objects:
                    if obj == object_ and ext in supported_formats:
                        image_files.append(join(obj_path, f'{obj}.{ext}'))
                        objects.remove(object_)

        for im_file in image_files:
            object_name = im_file.split('/')[-1].split('.')[0]

            if self.config.hold_prev_vals:
                self.object_timer[object_name] = time()

            try:
                object_im = cv2.imread(im_file)
                h, w = object_im.shape[0:2]
                obj_boundary = shrink_bbox([[0, 0], [0, h-1],
                                            [w-1, h-1], [w-1, 0]])

                if self.to_gray:
                    object_im = cv2.cvtColor(object_im, cv2.COLOR_BGR2GRAY)
                    object_im = object_im[obj_boundary[0, 1]:obj_boundary[2, 1],  # noqa: E501
                                          obj_boundary[0, 0]:obj_boundary[2, 0]]  # noqa: E501
                else:
                    object_im = object_im[obj_boundary[0, 1]:obj_boundary[2, 1],  # noqa: E501
                                          obj_boundary[0, 0]:obj_boundary[2, 0],  # noqa: E501
                                          :]

                kp, des = self.detector_descriptor.detectAndCompute(object_im, None)  # noqa: E501
                dim = object_im.shape[0:2]
                self.query_object_features[object_name] = [kp, des, dim]
            except:  # noqa: E722
                rospy.loginfo(f'Image couldn\'t be red at: \n {im_file}')

    def callback(self, sensor_image: np.ndarray):
        """
        Callback function for the object detection node

        Parameters
        ----------
        sensor_image : numpy.ndarray
            Image retrieved from a sensor (webcam/kinect).
        """
        image = np.frombuffer(
                sensor_image.data, dtype=np.uint8
            ).reshape(
                    sensor_image.height, sensor_image.width, -1
                )
        self.viz_frame = image
        if image is None:
            rospy.loginfo('invalid image received')
            return

        time_elapsed = time() - self.prev
        if time_elapsed > 1. / self.frame_rate:
            self.prev = time()
            for object_name, feat in self.query_object_features.items():
                self.detect(object_name, feat, image)

            # Convert the dictionary to string
            self.obj_boundary_msg = json.dumps(self.obj_boundary_info)
            self.obj_boundary_pub.publish(self.obj_boundary_msg)

            if self.config.show_image:
                cv2.imshow('Detected Objects', self.viz_frame)
                cv2.waitKey(10)

    def annotate_frame(self, viz_frame, dst, object_name):
        viz_frame = cv2.polylines(
                                    viz_frame,
                                    [dst],
                                    True,
                                    255,
                                    1,
                                    cv2.LINE_AA
                                )

        dst = np.squeeze(dst, axis=1)
        tc = (dst[3] + dst[0])/2
        tc = (tc + dst[0])/2

        text_loc = np.array([tc[0], tc[1] - 20], dtype=np.int16)
        base, tangent = dst[3] - dst[0]
        text_angle = np.arctan2(-tangent, base)*180/np.pi
        viz_frame = draw_angled_text(
                            object_name,
                            text_loc,
                            text_angle,
                            viz_frame
                        )
        return viz_frame

    def detect(
                self,
                object_name: str,
                query_img_feat: list,
                sensor_image: np.ndarray
            ):
        """
        Detects if the object is in the frame

        Parameters
        ----------
        object_name : str
            Name of the object.
        query_img_feat : list
            A list containing keypoints, descriptors, and
            dimension information of query object's image
        sensor_image : numpy.ndarray
            Image retrieved from a sensor (webcam/kinect).
        """
        MIN_MATCH_COUNT = self.config.min_match_count
        # If True the frame with detected object will
        # be showed, by default False
        show_image = self.config.show_image
        if self.to_gray:
            # sensor_rgb = sensor_image
            sensor_image = cv2.cvtColor(sensor_image, cv2.COLOR_BGR2GRAY)

        kp1, des1, dim = query_img_feat
        kp2, des2 = self.detector_descriptor.detectAndCompute(sensor_image, None)  # noqa: E501

        matches = self.matcher(des1, des2, **self.config.matcher_kwargs)
        # store all the good matches as per Lowe's ratio test.
        good = []
        for m, n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.float32(
                    [kp1[m.queryIdx].pt for m in good]
                ).reshape(-1, 1, 2)

            dst_pts = np.float32(
                    [kp2[m.trainIdx].pt for m in good]
                ).reshape(-1, 1, 2)
            M, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

            if M is None:
                return

            h, w = dim
            pts = np.float32(
                    [[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]
                ).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(pts, M).astype(np.int32)
            # update the location of the object in the image
            # converted to list as ndarray object is not json serializable
            self.obj_boundary_info[object_name] = np.squeeze(dst, axis=1).tolist()  # noqa: E501

            if self.config.hold_prev_vals:
                self.object_timer[object_name] = time()

            if show_image:
                # sensor_rgb = cv2.polylines(sensor_rgb, [dst] ,True,255,1, cv2.LINE_AA)  # noqa: E501
                self.viz_frame = self.annotate_frame(
                                            self.viz_frame,
                                            dst,
                                            object_name
                                        )

        else:
            if self.config.hold_prev_vals:
                if time() - self.object_timer[object_name] > self.config.hold_period:  # noqa: E501
                    self.obj_boundary_info[object_name] = None
                else:
                    if self.config.show_image and object_name in self.obj_boundary_info.keys():  # noqa: E501
                        self.viz_frame = self.annotate_frame(
                                            self.viz_frame,
                                            np.expand_dims(
                                                np.array(self.obj_boundary_info[object_name], dtype=np.int32),  # noqa: E501
                                                axis=1
                                            ),
                                            object_name
                                        )
            else:
                # Set None if the object isn't detected
                self.obj_boundary_info[object_name] = None
                rospy.loginfo("Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT))  # noqa: E501


if __name__ == '__main__':

    ObjectDetection(config=config)
