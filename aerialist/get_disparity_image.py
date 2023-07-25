#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2,os
from datetime import datetime
import numpy as np
import subprocess

folder = ""
sub_folder = ""
path = "/home/prasun/"

def ResizeWithAspectRatio(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv2.resize(image, dim, interpolation=inter)

def callback(data):
    current_datetime = datetime.now()
    str_current_datetime = str(current_datetime)
    bridge = CvBridge()
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    # rospy.loginfo(type(data.data))
    cv_image = bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
    resized_image = cv2.resize(cv_image,(800,800))
    normalized_image = cv2.normalize(resized_image, None, 0, 1, cv2.NORM_MINMAX, cv2.CV_32F)

    # rospy.loginfo(type(cv_image))
    # ims = cv2.resize(cv_image,(960,540))
    # rospy.loginfo(ims)
    # ims = ResizeWithAspectRatio(cv_image,width=1200)
    cv2.imwrite(path+"/disparity_image_"+str_current_datetime+".png",normalized_image*255.0)
    # with open("./msg.txt") as f:
    #     f.write(data.data)

    
def listener():
    global folder, sub_folder, path
    print(len(sys.argv))
    print(sys.argv)
    if(len(sys.argv)>=1):
        a=sys.argv[1]
        b = a.split('-')
        # print(b)

        folder = b[0]+"_"+b[1]
        # print(folder)

        if(len(b)>2):
            sub_folder = "_".join([str(item)for item in b[2:]])
            # print(sub_folder)

        if folder != "":
            if sub_folder!="":
                path += folder+"/"+sub_folder
            else:
                path += folder
        # print(path)
        isExist = os.path.exists(path)
        if isExist:
            current_datetime = datetime.now()
            str_current_datetime = str(current_datetime)
            new_sub_folder = str_current_datetime
            path = path+"/"+new_sub_folder
            os.makedirs(path)
        if not isExist:
            os.makedirs(path)
    rospy.init_node('disparity_image_listener', anonymous=True)
    rospy.Subscriber("/stereo/disparity_image", Image, callback)
    # rospy.Subscriber("/histogram_image", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()