import rospy
from sensor_msgs import Image
from std_msgs import *
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import argparse

def parse_args():
    pareser = argparse.ArgumentParser(
        prog='state_image.py'
        description='Publishes image of a given state'
    )
    parser.add_argument('img_folder', type=str, help='name of folder storing images', default = None, metavar = "Folder")
    args = parser.parse_args()
    return args

class StateImagePubSub(folder):
    def __init__(self):
        # Params
        self.br = CvBridge()
        self.image_left = cv2.imread(folder+'left_arrow.jpg', 1)
        self.image_right = cv2.imread(folder+'right_arrow.jpg',1)
        self.image_forward = cv2.imread(folder+'forward_arrow.jpg',1)
        self.image_goal = cv2.imread(folder+'goal_point.jpg',1)
        
        self.loop_rate = rospy.Rate(10)
        self.state_string = None

        self.pub = rospy.publish('state_image', Image, queue_size=10)
        rospy.Subscriber("state", String(), state_cb)
    
    def state_cb(self,msg):
        self.state_string = msg.data

    def loop(self):
        while not rospy.is_shutdown():
            if self.statestring == "left":
                self.pub.publish(br.cv2_to_imgmsg(self.image_left))
            elif self.statestring == "right":
                self.pub.publish(br.cv2_to_imgmsg(self.image_right))
            elif self.statestring == "goal":
                self.pub.publish(br.cv2_to_img_msgs(self.image_goal))
            else:
                self.pub.publish(br.cv2_to_img_msgs(self.image_forward))
            self.loop_rate.sleep()

if __name__ == "__main__":
    args = parse_args()
    rospy.init_node("image_state", anonymous=True)
    my_node = StateImagePubSub(folder=args.Folder)
    my_node.loop()
