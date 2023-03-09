import os
from time import sleep
from threading import Thread
import subprocess

# flag = True
#
#
# def task():
#     while flag:
#         os.system(
#             "/usr/bin/python2 /home/prasun/Downloads/capture_ros_topic_image/src/histogram_image/scripts/get_image.py")
#
#
# def printl():
#     global flag
#     for i in range(0, 5):
#         sleep(10)
#         print("Hello there")
#     flag = False
#
#
# thread = Thread(target=task)
# thread.start()
# printl()
# thread.join()

class Test:
    def __init__(self):
        image_command = "/usr/bin/python2 /home/prasun/Downloads/capture_ros_topic_image/src/histogram_image/scripts" \
                        "/get_image.py"
        self.image_process = subprocess.Popen(
            image_command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            preexec_fn=os.setsid,
            executable="/bin/bash",
        )
