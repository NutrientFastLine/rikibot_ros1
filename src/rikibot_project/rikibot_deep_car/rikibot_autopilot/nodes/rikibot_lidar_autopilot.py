#!/usr/bin/env python3

import rospy
import sys
import os
import cv2
import numpy as np
import tensorflow as tf
import h5py
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tensorflow.keras.models import load_model
from random import uniform
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from riki_msgs.msg import Sonar

class RikibotAutopilot: 
    def __init__(self):
        rospy.init_node("rikibot_autopilot", log_level=rospy.DEBUG)
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)


        self.linear_x = rospy.get_param('~speed', 0.5)
        self.detect_distance = rospy.get_param('~distance', 0.3)
        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = dir_path.replace('rikibot_autopilot/nodes', 'rikibot_autopilot/')
        self.model_path = dir_path + 'models/rikipolit'
        self.counter = 0
        self.obstacle_count = 0

        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320) # 分辨率
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240) # 分辨率
        cap.set(cv2.CAP_PROP_FPS, 20) # 帧数
        
        #rospy.loginfo("file: %s", dir_path)
        self.model = load_model(self.model_path)
        self.graph = tf.get_default_graph()
 
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.FindObstacle, queue_size=1)
        self.move_cmd = Twist()
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            img = cv2.resize(frame, (160, 120), cv2.INTER_LINEAR)
            last_image = np.asarray(img)
            with self.graph.as_default():
                last_image = last_image.reshape((1,) + last_image.shape)
                prediction = self.model.predict(last_image) 
                if self.obstacle_count > 5 : 
                    self.move_cmd.linear.x = 0
                    self.move_cmd.angular.z = 0
                else:
                    self.move_cmd.linear.x = self.linear_x
                    self.move_cmd.angular.z = prediction[0][0]
                self.pub_vel.publish(self.move_cmd)

                rospy.loginfo("Rikibot Autopilot speed linear_x: %f, angular_z : %s", self.move_cmd.linear.x, self.move_cmd.angular.z)
        r.sleep()


    def FindObstacle(self, scan):
        angle_scan = 30
        scan_start = 180 - angle_scan
        scan_end = 180 + angle_scan
        obstacle_existence = 'no'
        self.obstacle_count = 0
        for i in range(scan_start, scan_end):
            if scan.ranges[i] < self.detect_distance and scan.ranges[i] > 0.01:
                self.obstacle_count = self.obstacle_count + 1
            

    def main(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.loginfo("Rikibot Autopilot Start")
    node = RikibotAutopilot()
    node.main()
