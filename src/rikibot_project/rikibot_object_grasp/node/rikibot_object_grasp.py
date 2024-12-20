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
from riki_msgs.msg import Servo
from rikibot_nav import *
import eventlet
import time


class RikibotObjectFollower():
    def __init__(self):
        rospy.init_node('rikibot_trt_object_grasp_node', log_level=rospy.INFO)
        self.r = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown)
        self.linear_x = rospy.get_param('~speed', 0.1)
        self.detect_distance = rospy.get_param('~distance', 13)
        self.back_time = rospy.get_param('~back_time', 2)

        self.sonar_distance = 0
        self.image_center_x = 320
        self.offset_x = 60
        self.object_msg = None
        self.angular_speed_max = 1
	self.obj_grasp_pose =  rospy.get_param("~obj_grasp_pose", '2.245, 0.116, 0.000')
        self.obj_grasp_quat =  rospy.get_param("~obj_grasp_quat", '0.000, 0.000, -0.012, 1.000')
        self.obj_place_pose = rospy.get_param("~obj_place_pose", '3.537, 0.471, 0.000')
        self.obj_place_quat = rospy.get_param("~obj_place_quat", '0.000, 0.000, 0.688, 0.726')
        self.home = Pose(Point(0.000, 0.000, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.pub_servo = rospy.Publisher('/servo', Servo, queue_size=1)

        self.object_sub = rospy.Subscriber("/rikibot_detect_node/detections", Detection2DArray, self.ObjectCallback)
        self.sub_sonar = rospy.Subscriber('/sonar', Sonar, self.FindObstacle, queue_size=1)

        self.detect_time_end = self.detect_time_start =  rospy.get_time()
        self.x_pose = 150
        self.y_pose = 80
        self.timer = 0

        self.grasp_mode = False
        self.grasp_finish = False
        self.nav_lock = False
        self.nav_step = 0


        grasp_pose = list(map(float, self.obj_grasp_pose.split(',')))
        grasp_quat = list(map(float, self.obj_grasp_quat.split(',')))
        place_pose = list(map(float, self.obj_place_pose.split(',')))
        place_quat = list(map(float, self.obj_place_quat.split(',')))

        self.GraspGoal = Pose(Point(grasp_pose[0], grasp_pose[1], grasp_pose[2]), Quaternion(grasp_quat[0], grasp_quat[1], grasp_quat[2], grasp_quat[3]))
        self.PlaceGoal = Pose(Point(place_pose[0], place_pose[1], place_pose[2]), Quaternion(place_quat[0], place_quat[1], place_quat[2], place_quat[3]))
        
        self.servo = Servo()
        self.nav = RikibotNav()
        os.system("rostopic pub -1 servo riki_msgs/Servo -- '150' '80'")


        while not rospy.is_shutdown():
            self.detect_time_end =  rospy.get_time()
            if self.nav_lock is False:
                self.MoveSingleGrasp()
            else:
                if self.detect_time_end - self.detect_time_start < 1:
                    if self.object_msg is not None and self.grasp_mode is False and self.grasp_finish is False:
                        self.MoveCtrl()

                if self.grasp_mode is True:
                    self.GraspObject()

            self.servo.Servo1 =  self.x_pose
            self.servo.Servo2 =  self.y_pose
            self.pub_servo.publish(self.servo)
            self.r.sleep()


    def MoveSingleGrasp(self):
        if self.nav_step == 0:
            self.nav.SendGoal(self.GraspGoal)
        if self.nav_step == 1 and self.grasp_finish is True:
            self.nav.SendGoal(self.PlaceGoal)
        if self.nav_step == 2:
            self.GoHome()

        state =self.nav.NavState()
        if state == 1:
            self.nav_step = self.nav_step + 1
            if self.nav_step == 1:
                self.nav_lock  = True

    def GoHome(self):
        os.system("rostopic pub -1 servo riki_msgs/Servo -- '150' '80'")
        start_time = rospy.get_time()
        twist = Twist()
        while True:
            rospy.loginfo("Start Robot Back")
            end_time = rospy.get_time() - start_time
            twist.linear.x = -0.2
            twist.angular.z = 0
            self.vel_pub.publish(twist)
            self.r.sleep()
            if end_time > self.back_time:
                break

        rospy.loginfo("Start Robot GoHome")
        self.nav.SendGoal(self.home)

    def GraspObject(self):
        self.timer += 1
        if self.timer <= 20:
            self.y_pose = 100
        if 20 < self.timer <= 40:
            self.x_pose = 120
        if 40 <self.timer <= 60:
            self.y_pose = 80
        if self.timer > 60:
            self.grasp_mode = False
            self.grasp_finish = True
            self.nav_lock = False

        
    def ObjectCallback(self, object_msg):
        self.object_msg = object_msg
        self.detect_time_start = rospy.get_time()

    def MoveCtrl(self):
        twist = Twist()
        if (self.object_msg.detections[-1].bbox.center.x < self.image_center_x - self.offset_x) or (self.object_msg.detections[-1].bbox.center.x > self.image_center_x + self.offset_x):
            twist.angular.z = self.GetAngular(self.image_center_x - self.object_msg.detections[-1].bbox.center.x)
        else:
            if self.sonar_distance > self.detect_distance:
                twist.linear.x = self.linear_x
            else:
                self.grasp_mode = True
                twist.linear.x = 0
                twist.angular.z = 0

        rospy.loginfo("linear_x:%f, angular_z: %f", twist.linear.x,twist.angular.z)
        self.vel_pub.publish(twist)

    def FindObstacle(self, msg):
        self.sonar_distance = int(msg.distance)

    def GetAngular(self, offset):
        Kp = 0.002
        Kd = 0.0001
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

