#!/usr/bin/env python
from os.path import dirname, abspath
from os import chdir
import numpy as np
import message_filters
import struct
import json
import config

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

# from arm_pose.msg import Floats
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseStamped, PoseArray, Pose



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
        # Initializes ros node for planar pose estimation 
        rospy.init_node('planar_pose_estimation', anonymous=False)
        # Pose info gets published both as a `String` and
        # as `PoseArray`.
        self.pose_info_pub = rospy.Publisher('/object_pose_info', String, queue_size=10)
        self.pose_array_pub = rospy.Publisher('/object_pose_array', PoseArray, queue_size=10)
        
        self.object_pose_info = {}
        self.pose_array = PoseArray()
        # `self.obj_pose_info` is converted to json string and
        # then set to `self.obj_pose_msg` which then gets published
        self.obj_pose_msg = ''

     
        self.object_detection_sub = message_filters.Subscriber('/detected_object', String)
        self.pc_sub = message_filters.Subscriber(self.config.pc_sub['topic'], 
                                                 self.config.pc_sub['type'])

        ts = message_filters.ApproximateTimeSynchronizer([self.object_detection_sub, 
                                                          self.pc_sub], 10, 1, allow_headerless=True) # Changed code
        
        ts.registerCallback(self.callback)
        rospy.spin()
            
    def callback(self, object_detection_sub: str, pc_sub: PointCloud2):
        """
        Callback function for the planar pose estimation node

        Parameters
        ----------
        object_detection_sub : str
            A json string containing the information of the 
            bounding boxes of the detected objects
        pc_sub : PointCloud2
            A pointcloud object containing the 3D locations
            in terms of the frame `self.frame_id`
        """
        self.pose_array.header.frame_id = self.frame_id
        
        detected_object = json.loads(object_detection_sub.data)
        pose_array_msg = []
        for object_, bbox in detected_object.items():
            if bbox is not None:
                pose_msg = self.estimate_pose(object_, bbox, pc_sub)
                if pose_msg is not None:
                    pose_array_msg.append(pose_msg)
        
        self.pose_array.poses = pose_array_msg
        
        self.obj_pose_msg = json.dumps(self.object_pose_info)
        self.pose_info_pub.publish(self.obj_pose_msg)
        self.pose_array.header.stamp = rospy.Time.now()
        self.pose_array_pub.publish(self.pose_array)
        
    def estimate_pose(self, object_: str, bbox: list, pc_sub: PointCloud2):
        """
        Estimates planar pose of detected objects and 
        updates the stored pose.

        Parameters
        ----------
        object_: str
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
                    if object_ in self.object_pose_info.keys():
                        del self.object_pose_info[object_] 
                    rospy.loginfo('No corresponding 3D point found')
                    return
                else:
                    vectors_3D[pt_count] = dt
                    if pt_count is 2:
                        self.vectors_3D = vectors_3D
        except struct.error as err:
            rospy.loginfo(err)
            return
                
        # 3D position of the object
        c_3D = self.vectors_3D[0]

        # Center the vectors to the origin
        x_vec = self.vectors_3D[1] - c_3D
        y_vec = self.vectors_3D[2] - c_3D
        # Take the cross product of x and y vector
        # to generate z vector.
        z_vec = np.cross(x_vec, y_vec)
        # Recompute x vector to make it truly orthognal
        x_vec_orth = np.cross(y_vec, z_vec)
        
        # Normalize the orthogonal axes 
        try:
            x_vec_orth = x_vec_orth / np.linalg.norm(x_vec_orth)
            y_vec = y_vec / np.linalg.norm(y_vec)
            z_vec = z_vec / np.linalg.norm(z_vec)
        except RuntimeWarning as w:
            rospy.loginfo(w)
            return

        # Compute Euler angles i.e. roll, pitch, yaw
        roll = np.arctan2(y_vec[2], z_vec[2])
        pitch = np.arctan2(-x_vec_orth[2], np.sqrt(1 - x_vec_orth[2]**2))
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
        self.object_pose_info[object_] = {'position': c_3D.tolist(), 'orientation': [qx, qy, qz, qw]}
        
        return pose_msg
            
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