#!/usr/bin/env python
from os.path import dirname, abspath, join
from os import listdir, chdir
import numpy as np
import time
import cv2
import json

# Change directory to the file directory
dir_path = dirname(abspath(__file__))
chdir(dir_path)

# ROS imports
import rospy
from std_msgs.msg import String
import config
from utils import shrink_bbox


class ObjectDetection():
    def __init__(self, config, to_gray: bool=False):
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
        self.to_gray = to_gray
        self.detector_descriptor = config.detector_descriptor
        self.matcher = config.matcher
        
        # Holds all the information i.e. keypoints,  
        # descriptors, and dimension, for object detection.
        self.query_object_features = {}

        # Retrieves all the image feature info needed
        # for each of the object for detection
        self.extract_object_feat(self.config.objects, self.config.object_path)
        
        # Set frame rate
        self.frame_rate = 2
        self.prev = 0

        # Initializes ros node for object detection
        rospy.init_node('object_detection', anonymous=False)
        # `self.obj_boundary_info` is converted to json string and
        # then set to `self.obj_boundary_msg` which then gets published
        self.obj_boundary_msg = ''
        self.obj_boundary_info = {}
        self.obj_boundary_pub = rospy.Publisher('/detected_object', String, queue_size=10)

        # Starts the subscriber
        r = rospy.Rate(self.frame_rate)
        while not rospy.is_shutdown():
            rospy.Subscriber(self.config.image_sub['topic'], 
                             self.config.image_sub['type'], 
                             self.callback)
            r.sleep()
    
    def extract_object_feat(self, objects, obj_path: str):
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
        
        if objects is 'all':
            image_files = [join(obj_path, f) for f in listdir(obj_path) if f.endswith(('.jpg', '.png'))]
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
            try:
                object_im = cv2.imread(im_file)
                h, w = object_im.shape[0:2]
                obj_boundary = shrink_bbox([[0,0],[0, h-1],[w-1, h-1], [w-1, 0]])
                
                if self.to_gray:
                    object_im = cv2.cvtColor(object_im, cv2.COLOR_BGR2GRAY)
                    object_im = object_im[obj_boundary[0,1]:obj_boundary[2,1],
                                        obj_boundary[0,0]:obj_boundary[2,0]]
                else:
                    object_im = object_im[obj_boundary[0,1]:obj_boundary[2,1],
                                            obj_boundary[0,0]:obj_boundary[2,0],
                                            :]

                kp, des = self.detector_descriptor.detectAndCompute(object_im, None)
                dim = object_im.shape[0:2]
                self.query_object_features[object_name] = [kp, des, dim]                    # break
            except:
                rospy.loginfo(f'Image couldn\'t be red at: \n {im_file}')   
                
    def callback(self, sensor_image: np.ndarray):
        """
        Callback function for the object detection node

        Parameters
        ----------
        sensor_image : numpy.ndarray
            Image retrieved from a sensor (webcam/kinect).
        """
        image = np.frombuffer(sensor_image.data, dtype=np.uint8).reshape(sensor_image.height, sensor_image.width, -1)
        if image is None:
            rospy.loginfo('invalid image received')
            return
        
        time_elapsed = time.time() - self.prev
        if time_elapsed > 1. / self.frame_rate:
            self.prev = time.time()
            for object_name, feat in self.query_object_features.items():
                self.detect(object_name, feat, image)

            # Convert the dictionary to string
            self.obj_boundary_msg = json.dumps(self.obj_boundary_info)
            self.obj_boundary_pub.publish(self.obj_boundary_msg)

    def detect(self, object_name: str, query_img_feat: list, sensor_image: np.ndarray, show_image: bool=True):
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
        show_image : bool, optional
            If True the frame with detected object will 
            be showed, by default True
        """
        MIN_MATCH_COUNT = 10
        if self.to_gray:
            sensor_rgb = sensor_image
            sensor_image = cv2.cvtColor(sensor_image, cv2.COLOR_BGR2GRAY)

        kp1, des1, dim = query_img_feat
        kp2, des2 = self.detector_descriptor.detectAndCompute(sensor_image, None)

        matches = self.matcher(des1, des2, **self.config.matcher_kwargs)
        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            M, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            
            if M is None:
                return
            h,w = dim
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(pts,M).astype(np.int32)
            # update the location of the object in the image
            # converted to list as ndarray object is not json serializable
            self.obj_boundary_info[object_name] = np.squeeze(dst, axis=1).tolist()
            if show_image:
                if self.to_gray:
                    result = cv2.polylines(sensor_rgb, [dst] ,True,255,1, cv2.LINE_AA)
                else:
                    result = cv2.polylines(sensor_image, [dst] ,True,255,1, cv2.LINE_AA)
                cv2.imshow('Detected Objects', result)
                cv2.waitKey(10)
        else:
            # Set None if the object isn't detected
            self.obj_boundary_info[object_name] = None
            rospy.loginfo( "Not enough matches are found - {}/{}".
                format(len(good), MIN_MATCH_COUNT) )
        

if __name__ == '__main__':
    ObjectDetection(config=config)