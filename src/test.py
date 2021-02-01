from os.path import dirname, abspath, join
from os import listdir, chdir
import numpy as np
import time
import cv2
# import json

# Change directory to the file directory
dir_path = dirname(abspath(__file__))
chdir(dir_path)

# ROS imports

# from arm_pose.msg import Floats
from utils import shrink_bbox


def detect(object_name, query_im, kinect_im, desc, flann, show_image=True):
    now = time.time()
    # small_to_large_image_size_ratio = 0.9
    # kinect_im = cv2.cvtColor(kinect_im, cv2.COLOR_BGR2GRAY)
    # kinect_im = cv2.resize(kinect_im, # original image
    #                     (0,0), # set fx and fy, not the final size
    #                     fx=small_to_large_image_size_ratio, 
    #                     fy=small_to_large_image_size_ratio, 
    #                     interpolation=cv2.INTER_NEAREST)
    # query_im = cv2.cvtColor(query_im, cv2.COLOR_BGR2GRAY)
    # minimum matching points needed to consider a match
    MIN_MATCH_COUNT = 10
    
    # img1 = cv.imread('box.png',0)          # queryImage
    # img2 = cv.imread('box_in_scene.png',0) # trainImage

    kp2, des2 = desc.detectAndCompute(kinect_im,None)

    # matches = flann.knnMatch(des1, des2, k=2)
    matches = flann.knnMatch(des1,des2, k=2)
    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        print(f'Time is: {time.time() - now}')
        
        if M is None:
            return
        # matchesMask = mask.ravel().tolist()
        h,w = query_im.shape[0:2]
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts,M).astype(np.int32)
        # update the location of the object in the image
        # converted to list as ndarray object is not json serializable
        print(f'Before Shrink: {time.time() - now}')
        # dst2 = shrink_bbox(dst[:, 0, :], shrink_val=0.9).tolist()
        print(f'After Shrink: {time.time() - now}')

        if show_image:
            kinect_im = cv2.polylines(kinect_im, [dst],True,255,1, cv2.LINE_AA)
            print(f'Draw time: {time.time() - now}')

            cv2.imshow('Detected Objects', kinect_im)
            cv2.waitKey(10)
            
    else:
        # Set None if the object isn't detected
        # matchesMask = None
        print('Not enough points')
        


vid = cv2.VideoCapture(0) 

# Initiate SIFT detector
sift = cv2.SIFT_create()
# brisk = cv2.BRISK_create()
desc = sift
# find the keypoints and descriptors with SIFT
FLANN_INDEX_KDTREE = 1

index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
bf = cv2.BFMatcher()

flann = cv2.FlannBasedMatcher(index_params, search_params)
query_im = cv2.imread('objects/book-1.jpg')
h, w = query_im.shape[0:2]
reduce_bbox = shrink_bbox([[0,0],[0,h],[w,h],[w,0]])
query_im = query_im[reduce_bbox[0,1]:reduce_bbox[2,1], 
                    reduce_bbox[0,0]:reduce_bbox[2,0], :]
cv2.imshow('q', query_im)
# query_im = cv2.cvtColor(query_im, cv2.COLOR_BGR2GRAY)
cv2.waitKey(10)

kp1, des1 = desc.detectAndCompute(query_im,None)

frame_rate = 10
prev = 0
count = 0
while True:
    time_elapsed = time.time() - prev

    ret, frame = vid.read() 
    # if time_elapsed > 1./frame_rate:
    #     prev = time.time()  
    #     detect('book', query_im, frame, desc, flann)
    if count % 30 is 0:
        count = 0
    detect('book', query_im, frame, desc, flann)
        
    count += 1
