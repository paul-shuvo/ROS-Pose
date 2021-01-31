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

# ROS imports
import rospy
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2

# from arm_pose.msg import Floats
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseStamped, PoseArray, Pose



class PlanarPoseEstimation():
    def __init__(self, config):
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
            
    def callback(self, object_detection_sub: String, pc_sub: PointCloud2):
        """
        Callback function for the planar pose estimation node

        Parameters
        ----------
        object_detection_sub : String
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
        
    def estimate_pose(self, object_: String, bbox: list, pc_sub: PointCloud2):
        """
        Estimates planar pose of detected objects and 
        updates the stored pose.

        Parameters
        ----------
        bbox : list
            Contains the coordinates of the bounding box
            of the detected object.
        pc_sub : PointCloud2
            A pointcloud object containing the 3D locations
            in terms of the frame `self.frame_id`
        """
        
        bbox = np.array(bbox)
        c = (bbox[0] + bbox[2]) // 2
        x = (bbox[2] + bbox[3]) // 2
        y = (bbox[0] + bbox[3]) // 2
        
        # center = [(bbox[0][0] + bbox[2][0]) // 2 , (bbox[0][1] + bbox[2][1]) // 2]
        points = np.array([c, x, y]).tolist()
        print(points)
        # print(bbox)
        vectors_3D = np.zeros((3,3))
        try:
            for pt_count, dt in enumerate(pc2.read_points(pc_sub, field_names={'x','y','z'}, skip_nans=False, uvs=points)):
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
                
        c_3D = self.vectors_3D[0]
        x_vec = self.vectors_3D[1] - c_3D
        y_vec = self.vectors_3D[2] - c_3D
        z_vec = np.cross(x_vec, y_vec)
        x_vec_orth = np.cross(y_vec, z_vec)
        
        x_vec_orth = x_vec_orth / np.linalg.norm(x_vec_orth)
        y_vec = y_vec / np.linalg.norm(y_vec)
        z_vec = z_vec / np.linalg.norm(z_vec)

        roll = np.arctan2(y_vec[2], z_vec[2])
        pitch = np.arctan2(-x_vec_orth[2], np.sqrt(1 - x_vec_orth[2]**2))
        yaw = np.arctan2(x_vec_orth[1], x_vec_orth[0])
        
        [qx, qy, qz, qw] = self.euler_to_quaternion(roll, pitch, yaw)
        
        pose_msg = Pose()
        
        # pose_msg.header.frame_id = self.frame_id
        # pose_msg.header.stamp = rospy.Time.now()
        pose_msg.position.x = c_3D[0]
        pose_msg.position.y = c_3D[1]
        pose_msg.position.z = c_3D[2]
        # Make sure the quaternion is valid and normalized
        pose_msg.orientation.x = qx
        pose_msg.orientation.y = qy
        pose_msg.orientation.z = qz
        pose_msg.orientation.w = qw
        
        self.object_pose_info[object_] = {'position': c_3D.tolist(), 'orientation': [qx, qy, qz, qw]}
        
        return pose_msg
            # vectors_3D[pt_count]
            
    def euler_to_quaternion(self, roll: float, pitch: float, yaw:float):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
if __name__ == "__main__":
    PlanarPoseEstimation(config=config)