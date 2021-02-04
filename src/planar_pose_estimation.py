#!/usr/bin/env python
from os.path import dirname, abspath
from os import chdir
import numpy as np
from numpy.linalg import norm
from numpy.core.arrayprint import dtype_is_implied
import message_filters
import struct
import json
import cv2
import config
from utils import draw_angled_text

# Change directory to the file directory
dir_path = dirname(abspath(__file__))
chdir(dir_path)

# Except warnings as errors
import warnings
warnings.filterwarnings("error")


# ROS imports
import rospy
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from image_geometry import PinholeCameraModel

# from arm_pose.msg import Floats
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

# TODO: Add doc for viz_pose
# TODO: Add roatated text feature
# TODO: Test the code more
# FIX: The long blue axis 

class PlanarPoseEstimation():
    def __init__(self, config):
        """
        Generates poses for detected objects.

        Parameters
        ----------
        config : object
            Contains the configuration for the class.
        """
        self.config = config
        self.frame_id = self.config.frame_id
        self.viz_pose = config.viz_pose
        # Initializes ros node for planar pose estimation 
        rospy.init_node('planar_pose_estimation', anonymous=False)
        # Pose info gets published both as a `String` and
        # as `PoseArray`.
        self.pose_info_pub = rospy.Publisher('/object_namepose_info', String, queue_size=10)
        self.pose_array_pub = rospy.Publisher('/object_namepose_array', PoseArray, queue_size=10)
        
        self.object_namepose_info = {}
        self.pose_array = PoseArray()
        # `self.obj_pose_info` is converted to json string and
        # then set to `self.obj_pose_msg` which then gets published
        self.obj_pose_msg = ''

     
        self.object_namedetection_sub = message_filters.Subscriber('/detected_object', String)
        self.pc_sub = message_filters.Subscriber(self.config.pc_sub['topic'], 
                                                 self.config.pc_sub['type'])
        self.image_sub = message_filters.Subscriber(self.config.image_sub['topic'], 
                                                    self.config.image_sub['type'])

        if self.viz_pose:
            self.viz_frame = None
            camera_info = rospy.wait_for_message(self.config.cam_info_sub['topic'], 
                                                 self.config.cam_info_sub['type'])
            self.P = np.array(camera_info.P).reshape((3, 4))



        ts = message_filters.ApproximateTimeSynchronizer([self.object_namedetection_sub, 
                                                          self.pc_sub, self.image_sub], 
                                                          10, 1, allow_headerless=True)
        
        ts.registerCallback(self.callback)
        rospy.spin()
            
    def callback(self, object_namedetection_sub: str, pc_sub: PointCloud2, image_sub: Image):
        """
        Callback function for the planar pose estimation node

        Parameters
        ----------
        object_namedetection_sub : str
            A json string containing the information of the 
            bounding boxes of the detected objects
        pc_sub : PointCloud2
            A pointcloud object containing the 3D locations
            in terms of the frame `self.frame_id`
        """
        if self.viz_pose:
            self.viz_frame = np.frombuffer(image_sub.data, dtype=np.uint8).reshape(image_sub.height, image_sub.width, -1)

        self.pose_array.header.frame_id = self.frame_id
        
        detected_object = json.loads(object_namedetection_sub.data)
        pose_array_msg = []
        for object_name, bbox in detected_object.items():
            if bbox is not None:
                pose_msg = self.estimate_pose(object_name, bbox, pc_sub)
                if pose_msg is not None:
                    pose_array_msg.append(pose_msg)
        
        if self.viz_pose:
            cv2.imshow('Pose', self.viz_frame)
            cv2.waitKey(1)
            
        self.pose_array.poses = pose_array_msg
        
        self.obj_pose_msg = json.dumps(self.object_namepose_info)
        self.pose_info_pub.publish(self.obj_pose_msg)
        self.pose_array.header.stamp = rospy.Time.now()
        self.pose_array_pub.publish(self.pose_array)
        
        
    def estimate_pose(self, object_name: str, bbox: list, pc_sub: PointCloud2):
        """
        Estimates planar pose of detected objects and 
        updates the stored pose.

        Parameters
        ----------
        object_name: str
            Name of the object.
        bbox : list
            Contains the coordinates of the bounding box
            of the detected object.
        pc_sub : PointCloud2
            A pointcloud object containing the 3D locations
            in terms of the frame `self.frame_id`
        """
        
        bbox = np.array(bbox)
        
        # Compute the center, the mid point of the right 
        # and top segment of the bounding box  
        c = (bbox[0] + bbox[2]) // 2
        x = (bbox[2] + bbox[3]) // 2
        y = (bbox[0] + bbox[3]) // 2
        
        points = np.array([c, x, y]).tolist()
        vectors_3D = np.zeros((3,3))
        try:
            # Get the corresponding 3D location of c, x, y
            for pt_count, dt in enumerate(pc2.read_points(pc_sub, field_names={'x','y','z'}, skip_nans=False, uvs=points)):
                # If any point returns nan, return
                if np.any(np.isnan(dt)):
                    if object_name in self.object_namepose_info.keys():
                        del self.object_namepose_info[object_name] 
                    rospy.loginfo('No corresponding 3D point found')
                    return
                else:
                    vectors_3D[pt_count] = dt
                    if pt_count is 2:
                        self.vectors_3D = vectors_3D
        except struct.error as err:
            rospy.loginfo(err)
            return
        
        try:        
            # 3D position of the object
            c_3D = self.vectors_3D[0]
            
            # Center the vectors to the origin
            x_vec = self.vectors_3D[1] - c_3D
            x_vec /= norm(x_vec)
            
            y_vec = self.vectors_3D[2] - c_3D
            y_vec /= norm(y_vec)
            # Take the cross product of x and y vector
            # to generate z vector.
            z_vec = np.cross(x_vec, y_vec)
            z_vec /= norm(z_vec)
            
            # Recompute x vector to make it truly orthognal
            x_vec_orth = np.cross(y_vec, z_vec)
            x_vec_orth /= norm(x_vec_orth)
        except RuntimeWarning as w:
            rospy.loginfo(w)
            return
        
        if self.viz_pose:
            self.draw_pose(object_name, np.vstack((self.vectors_3D, z_vec)))
        
        # Compute Euler angles i.e. roll, pitch, yaw
        roll = np.arctan2(y_vec[2], z_vec[2])
        pitch = np.arctan2(-x_vec_orth[2], np.sqrt(1 - x_vec_orth[2]**2))
        # pitch = np.arcsin(-x_vec_orth[2])
        yaw = np.arctan2(x_vec_orth[1], x_vec_orth[0])
                
        # Convert to quaternion
        [qx, qy, qz, qw] = self.euler_to_quaternion(roll, pitch, yaw)
        
        # Generate Pose message.
        pose_msg = Pose()
        
        pose_msg.position.x = c_3D[0]
        pose_msg.position.y = c_3D[1]
        pose_msg.position.z = c_3D[2]
        # Make sure the quaternion is valid and normalized
        pose_msg.orientation.x = qx
        pose_msg.orientation.y = qy
        pose_msg.orientation.z = qz
        pose_msg.orientation.w = qw
        
        # Store/update the pose information.
        self.object_namepose_info[object_name] = {'position': c_3D.tolist(), 'orientation': [qx, qy, qz, qw]}
        
        return pose_msg
    
    def draw_pose(self, object_name: str, vectors_3D: np.ndarray):
        """
        Draw poses as directional axes on the image.

        Parameters
        ----------
        object_name : str
            Name of the object
        vectors_3D : numpy.ndarray
            The 3D directional vectors.
        """

        p_image = np.zeros((4,2), dtype=np.int32)
        coordinates = None
        for i, vec in enumerate(vectors_3D):
            coordinates = self.project3dToPixel(vec)
            if coordinates.any() is None:
                break
            # print(f'pixel is: {coordinates}')
            p_image[i]= coordinates
        
        if coordinates is not None:
            p_image[3] = p_image[0] - (p_image[3] - p_image[0])
            # z = c + (z-c)*(norm(x-c)/norm(z-c))
            p_image[3] = p_image[0] + (p_image[3] - p_image[0])*(norm(p_image[1] - p_image[0])/norm(p_image[3] - p_image[0]))
            colors_ = [(0,0,255),(0,255,0),(255,0,0)]
            for i in range(1,4):
                cv2.line(self.viz_frame, tuple(p_image[0]), tuple(p_image[i]), colors_[i-1], thickness=2)
                x1, y1, x2, y2 = self.calc_vertexes(p_image[0], p_image[i])
                cv2.line(self.viz_frame, tuple(p_image[i]), (x1, y1), colors_[i-1], thickness=2)
                cv2.line(self.viz_frame, tuple(p_image[i]), (x2, y2), colors_[i-1], thickness=2)

            # Put object label aligned to the object's in-plane planar rotation            
            text_loc = np.array([p_image[2,0] - (p_image[1,0] - p_image[0,0])/2, p_image[2,1] - 20], dtype=np.int16)
            base, tangent = p_image[1] - p_image[0]
            text_angle = np.arctan2(-tangent, base)*180/np.pi
            self.viz_frame = draw_angled_text(object_name, text_loc, text_angle, self.viz_frame)

            
    def project3dToPixel(self, point):
        """
        Find the 3D point projected to image plane

        Parameters
        ----------
        point : numpy.ndarrya | tuple | list
            The 3D point.
        Returns
        -------
        numpy.ndarray | None
            The pixel location corresponding to the 
            3D vector. Returns None if w is 0.
        """
        src = np.array([point[0], point[1], point[2], 1.0]).reshape(4,1)
        dst = self.P @ src
        x = dst[0,0]
        y = dst[1,0]
        w = dst[2,0]
        if w != 0:
            px = int(x/w)
            py = int(y/w)
            return np.array([px, py], dtype=np.int32)
        else:
            return None
    
    def calc_vertexes(self, start_cor: np.ndarray, end_cor: np.ndarray):
        """
        Calculate line segments of the vector arrows to be drawn.

        Parameters
        ----------
        start_cor : numpy.ndarray
            Base point of the arrow.
        end_cor : numpy.ndarray
            End point of the arrow.

        Returns
        -------
        list
            Location of the edge of arrow segments.
        """
        start_x, start_y = start_cor
        end_x, end_y = end_cor
        angle = np.arctan2(end_y - start_y, end_x - start_x) + np.pi
        arrow_length = 15
        arrow_degrees_ = 70

        x1 = int(end_x + arrow_length * np.cos(angle - arrow_degrees_)) 
        y1 = int(end_y + arrow_length * np.sin(angle - arrow_degrees_))
        x2 = int(end_x + arrow_length * np.cos(angle + arrow_degrees_))
        y2 = int(end_y + arrow_length * np.sin(angle + arrow_degrees_))

        return x1, y1, x2, y2
    
    def euler_to_quaternion(self, roll: float, pitch: float, yaw:float):
        """
        Converts euler angles to quaternion

        Parameters
        ----------
        roll : float
            Roll angle.
        pitch : float
            Pitch angle.
        yaw : float
            Yaw angle.

        Returns
        -------
        list
            Converted Quaternion values.
        """

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
    
if __name__ == "__main__":
    PlanarPoseEstimation(config=config)