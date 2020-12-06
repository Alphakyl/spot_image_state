#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class StateImagePubSub():
    def __init__(self, folder):
        # Params
        self.br = CvBridge()
        self.image_left = cv2.imread('/home/arpg/spot_ws/src/spot_image_state/img/right_arrow.jpg', 1)
        self.image_right = cv2.imread('/home/arpg/spot_ws/src/spot_image_state/img/left_arrow.jpg',1)
        self.image_forward = cv2.imread('/home/arpg/spot_ws/src/spot_image_state/img/forward_arrow.jpg',1)
        self.image_goal = cv2.imread('/home/arpg/spot_ws/src/spot_image_state/img/goal_point.jpg',1)
        # print type(self.image_left)

        self.loop_rate = rospy.Rate(10)
        self.statestring = None

        self.pub = rospy.Publisher('state_image', Image, queue_size=10)
        rospy.Subscriber('/spot_joy_interface/state', String, self.state_cb)
    
    def state_cb(self,msg):
        self.statestring = msg.data

    def loop(self):
        while not rospy.is_shutdown():
            if self.statestring == "left_turn":
                self.pub.publish(self.br.cv2_to_imgmsg(self.image_left))
            elif self.statestring == "right_turn":
                self.pub.publish(self.br.cv2_to_imgmsg(self.image_right))
            elif self.statestring == "at_goal":
                self.pub.publish(self.br.cv2_to_imgmsg(self.image_goal))
            else:
                self.pub.publish(self.br.cv2_to_imgmsg(self.image_forward))
            self.loop_rate.sleep()

if __name__ == "__main__":
    rospy.init_node("image_state", anonymous=True)
    my_node = StateImagePubSub(folder='/home/arpg/spot_ws/spot_image_state/img/')
    my_node.loop()
