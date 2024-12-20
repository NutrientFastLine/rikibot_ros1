#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy
import numpy as np
import os,cv2
import math
import imutils
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import BoundingBox2D, Detection2DArray
from geometry_msgs.msg import Twist
from riki_msgs.msg import Sonar

class RikibotObjectFollower():
    def __init__(self):
        rospy.init_node('rikibot_people_follower_node', log_level=rospy.INFO)
        r = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown)
        self.linear_x = rospy.get_param('~speed', 0.1)
        self.detect_distance = rospy.get_param('~distance', 15)


        self.object_center_x = self.image_center_x = 320
        self.sonar_distance = 0
        self.image_center_y = 240
        self.offset_x = 80
        self.offset_rect = 400
        self.angular_speed_max = 1

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.object_sub = rospy.Subscriber("/detect_bbox", BoundingBox2D, self.ObjectCallback)
        self.sub_sonar = rospy.Subscriber('/sonar', Sonar, self.FindObstacle, queue_size=1)
        os.system("rostopic pub -1 servo riki_msgs/Servo -- '150' '80'")
        self.detect_time_end = self.detect_time_start =  rospy.get_time()
        #rospy.Timer(rospy.Duration(1), self.DetectTimerCallback)
        while not rospy.is_shutdown():
            self.detect_time_end =  rospy.get_time()
            if self.detect_time_end - self.detect_time_start < 1:
                self.MoveCtrl()
            r.sleep()


    def MoveCtrl(self):
        twist = Twist()
        if (self.object_center_x < self.image_center_x - self.offset_x) or (self.object_center_x > self.image_center_x + self.offset_x):
            twist.angular.z = self.GetAngular(self.image_center_x - self.object_center_x)
        else:
            if self.sonar_distance > self.detect_distance:
                twist.linear.x = self.linear_x
            else:
                os.system("rostopic pub -1 servo riki_msgs/Servo -- '150' '100'")
                os.system("rostopic pub -1 servo riki_msgs/Servo -- '120' '100'")
                os.system("rostopic pub -1 servo riki_msgs/Servo -- '120' '60'")
                twist.linear.x = 0
                twist.angular.z = 0
 
        rospy.loginfo("linear_x:%f, angular_z: %f", twist.linear.x,twist.angular.z)  
        #twist.linear.x = 0
        #twist.angular.z =angular_z
        self.vel_pub.publish(twist)


    def ObjectCallback(self, object_msg):
        self.detect_time_start = rospy.get_time()

        #rospy.loginfo(object_msg.detections[-1].bbox)  
        #rospy.loginfo("size: %f, center: %f", object_msg.size_x, object_msg.center.x)  
        self.object_center_x = object_msg.center.x



    def FindObstacle(self, msg):
        self.sonar_distance = int(msg.distance)


    def GetAngular(self, offset):
        Kp = 0.003
        Kd = 0.0002
        last_error = 0
        error = offset
        if abs(error) >= 320:
            error = last_error

        speed = Kp*error + Kd*(error - last_error)
        last_error = error
        if speed > self.angular_speed_max:
            speed = self.angular_speed_max

        if speed < -self.angular_speed_max:
            speed = -self.angular_speed_max

        return speed


    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown face_location_node ...")

if __name__ == '__main__':
    try:
        detect = RikibotObjectFollower()
        rospy.spin()
    except Exception, e:
        rospy.logerr("%s", str(e))
        os._exit(1)

