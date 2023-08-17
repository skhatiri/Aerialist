#!/usr/bin/env python
import sys
import rospy
# from decouple import config
# from dotenv import load_dotenv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, os
from datetime import datetime

# load_dotenv()
folder = ""
sub_folder = ""
path = "/src/"
# path = "/home/prasun/"
# # path = config("IMAGE_PATH")
# path = os.getenv("IMAGE_PATH")

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
    cv_image = bridge.imgmsg_to_cv2(data)
    # rospy.loginfo(type(cv_image))
    # ims = cv2.resize(cv_image,(960,540))
    ims = ResizeWithAspectRatio(cv_image, width=1200)
    cv2.imwrite(path + "/raw_image_" + str_current_datetime + ".png", ims)
    # with open("./msg.txt") as f:
    #     f.write(data.data)


def listener():
    global folder, sub_folder, path
    a = "raw_image"
    b = a.split('_')
    # print(b)

    folder = b[0] + "_" + b[1]
    # print(folder)

    if (len(b) > 2):
        sub_folder = "_".join([str(item) for item in b[2:]])
        # print(sub_folder)

    if folder != "":
        if sub_folder != "":
            path += folder + "/" + sub_folder
        else:
            path += folder
    # print(path)
    isExist = os.path.exists(path)
    if isExist:
        current_datetime = datetime.now()
        str_current_datetime = str(current_datetime)
        new_sub_folder = str_current_datetime
        path = path + "/" + new_sub_folder
        os.makedirs(path)
    if not isExist:
        os.makedirs(path)
    rospy.init_node('raw_image_listener', anonymous=True)
    rospy.Subscriber("/stereo/left/image_rect_color", Image, callback)
    # rospy.Subscriber("/histogram_image", Image, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
