#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import os,sys
#sys.path.remove('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))
import cv2
#sys.path.append('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO'))) 
#roslib.load_manifest("rikibot_controller")
import json
from std_msgs.msg import UInt8
from shutil import copy, rmtree
from geometry_msgs.msg import Twist
from collections import OrderedDict


class TrainController():
    def __init__(self):
    
	self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)

	self.mode = "auto"
        self.ixcount = 1
        self.image_name = "_cam-image_array_.jpg"
	self.steeringAngle = 0.0
	self.throttle = 0.0
        self.save_dir = "/home/rikibot/Work/train_data"
       
	cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320) # 分辨率
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240) # 分辨率
        cap.set(cv2.CAP_PROP_FPS, 20) # 帧数
 


        rospy.Subscriber('/cmd_vel', Twist, self.TwistVel)
        if os.path.exists(self.save_dir):
            rmtree(self.save_dir)
        os.makedirs(self.save_dir)
        copy("/home/rikibot/catkin_ws/src/rikibot_project/rikibot_deep_car/install/meta.json", self.save_dir + "/")
            

        while not rospy.is_shutdown():
	    ret, frame = cap.read()
            img = cv2.resize(frame, (160, 120), cv2.INTER_LINEAR)
            json_data = OrderedDict()
 
            
            #if self.cv_image != None:
            if self.throttle != 0:
                cv2.imwrite(self.save_dir + "/" + str(self.ixcount)+ self.image_name, img)
                json_data["user/mode"] = "user"
                json_data['cam/image_array'] = str(self.ixcount)+ self.image_name
                json_data['user/throttle'] = self.throttle
                json_data['user/angle'] = self.steeringAngle
                json.dumps(json_data)
                path = self.save_dir + "/record_" + str(self.ixcount) + ".json" 
                try:
                    with open(path, 'w') as fp: 
                        json.dump(json_data, fp) 
                except TypeError:
                    logger.warn('troubles with record: {}'.format(json_data))

                self.ixcount = self.ixcount + 1
	    r.sleep()


    def TwistVel(self, twist_msg):		
        self.throttle = round(twist_msg.linear.x, 1)
	self.steeringAngle = round(twist_msg.angular.z, 1)


    def SaveTrainImage(self, image_msg):
        if self.sub_image_type == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            size = image.shape
        elif self.sub_image_type == "raw":
            image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
            size = image.shape
        self.cv_image = cv2.resize(image, (size[1]/2, size[0]/2), cv2.INTER_LINEAR)

        
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('TrainController')
    node = TrainController()
    node.main()

