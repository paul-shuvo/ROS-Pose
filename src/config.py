from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import cv2

# Reference frame for pose estimation
frame_id = 'kinect2_rgb_optical_frame'

# Image topic for object detection
image_sub = {
    'topic': '/kinect2/qhd/image_color_rect',
    'type': Image
    }

# Pointcloud topic for retrieving
# 3D locations
pc_sub = {
    'topic': '/kinect2/qhd/points',
    'type': PointCloud2
    }

# CameraInfo topic for retrieving
# camera matrix

cam_info_sub = {
    'topic': '/kinect2/qhd/camera_info',
    'type': CameraInfo
    }

# Make sure all the topics corrsponds
# image resolution, i.e. qhd, sd, or hd
assert image_sub['topic'].split('/')[2] == pc_sub['topic'].split('/')[2], 'image topic and point cloud topic have resolution mismatch'
assert image_sub['topic'].split('/')[2] == cam_info_sub['topic'].split('/')[2], 'image topic and camera info topic have resolution mismatch'


# Path containing images of query objects
object_path = 'objects'
# Name of the objects corresponding to
# the files name without the extension
# e.g. book-1.jpg corresponds to book-1
objects = ['book-1']

# Minimum match required for an object to be considered detected
min_match_count = 15

# Define detector-descriptor and the 
# matcher algortihm
detector_descriptor = cv2.SIFT_create()
matcher = cv2.FlannBasedMatcher(dict(algorithm=1, trees=5),
                                dict(checks=50)).knnMatch
# If there's no kwargs for the matcher keep
# it as an empty `dict` e.g. {}
matcher_kwargs = {'k':2}

# Visualize the detected objects,
# if set to True
show_image = True

# Visualize the pose of the objects, 
# if set to True
viz_pose = True