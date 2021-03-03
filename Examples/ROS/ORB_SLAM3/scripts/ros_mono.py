#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
from sensor_msgs.msg import Image

import sys, os
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


def mainFunc():
    rospy.init_node('GetImage', anonymous=True)
    rospy.loginfo("Here we go.")
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    image = Image()
    bridge = CvBridge()

    # set the video reader
    video_path = 1 # camera number index
    # video_path = "/home/pacific/Documents/Work/Projects/Workflows/server/PycharmProjects/Pacific_AvatarGame_Host/utils/camera/test.mp4" # real video file

    if type(video_path).__name__ == "str":
        videoReader = cv2.VideoCapture(video_path)
        print("Load live video from file...")
    elif type(video_path).__name__ == "int":
        videoReader = cv2.VideoCapture(video_path)
        print("Get live video from camera...")

    if videoReader.isOpened():
        print("Camera status ready...")
    else:
        print("Camera status fault...")
        exit()

    video_fps = videoReader.get(cv2.CAP_PROP_FPS)
    print("Live Video FPS: ", video_fps)
    rate = rospy.Rate(video_fps) # 50hz

    frame_width = videoReader.get(cv2.CAP_PROP_FRAME_WIDTH)
    frame_height = videoReader.get(cv2.CAP_PROP_FRAME_HEIGHT)
    frame_size = (int(frame_width), int(frame_height))
    print("Live Video Frame Size: ", frame_size)
    
    # read and write the video frame
    while videoReader.isOpened():
        # get the video frame
        success, frame = videoReader.read()

        if success:
            # show the video frame
            # print("Live Video Frame Shape: {}".format(frame.shape))
            # cv2.putText(frame, "Live Video Stream", (400,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
            cv2.namedWindow("Live Video", cv2.WINDOW_NORMAL)
            cv2.imshow("Live Video", frame)

            # publish the image
            # image.header.stamp = rospy.Time.now()
            # image.height = int(frame_height)
            # image.width = int(frame_width)
            # image.encoding = "rgb16"
            # image.is_bigendian = 0
            # image.step = 640
            # image.data = frame.flatten().tolist()

            image = bridge.cv2_to_imgmsg(frame, "bgr8")

            pub.publish(image)
            rate.sleep()

            # check whether manual exit command entered
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            else:
                continue
    
    # release the objects and exit
    videoReader.release()
    cv2.destroyAllWindows()

    print("Live Video Done.")


if __name__ == '__main__':
    mainFunc()
