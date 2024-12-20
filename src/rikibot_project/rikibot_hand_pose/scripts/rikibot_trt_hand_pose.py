#! /usr/bin/env python3
import rospy
import os
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
import json
import trt_pose.coco
import trt_pose.models
import torch
import torch2trt
from torch2trt import TRTModule
import cv2
import torchvision.transforms as transforms
import numpy as np
from trt_pose.draw_objects import DrawObjects
from trt_pose.parse_objects import ParseObjects
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from preprocessdata import preprocessdata
import PIL.Image
import pickle
#from rikibot_hand_pose.msg import *


class RikibotPose:
    def __init__(self):

        self.image_pub = rospy.Publisher("/rikibot_hand_pose/img_np",Image,queue_size=1)

        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = dir_path.replace('rikibot_hand_pose/scripts', 'rikibot_hand_pose/')

        self.model = dir_path + 'models/trt_hand_pose_resnet18_att_244_244.pth'
        self.pose_config = dir_path +'models/hand_pose.json'

        with open(self.pose_config, 'r') as f:
            self.hand_pose = json.load(f)

        self.topology = trt_pose.coco.coco_category_to_topology(self.hand_pose)
        self.num_parts = len(self.hand_pose['keypoints'])
        self.num_links = len(self.hand_pose['skeleton'])
        self.model_trt = TRTModule()
        rospy.loginfo("load model")
        self.model_trt.load_state_dict(torch.load(self.model))
        self.mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
        self.std = torch.Tensor([0.229, 0.224, 0.225]).cuda()
        self.device = torch.device('cuda')
        rospy.loginfo("load model done")

        self.parse = ParseObjects(self.topology, cmap_threshold=0.12, link_threshold=0.15)
        self.draw = DrawObjects(self.topology)
        self.preprocessdata = preprocessdata(self.topology, self.num_parts)
        self.clf = make_pipeline(StandardScaler(), SVC(gamma='auto', kernel='rbf'))

        filename = dir_path + 'models/svmmodel.sav'
        self.clf = pickle.load(open(filename, 'rb'))
        self.gesture_cfg = dir_path + 'models/gesture.json'
        with open(self.gesture_cfg, 'r') as f:
            gesture = json.load(f)
            self.gesture_type = gesture["classes"]

        self.cam = cv2.VideoCapture(0)
        self.rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            ret, image = self.cam.read()
            image = cv2.resize(image, (224,224))
            self.execute(image)
            self.rate.sleep()

    def preprocess(self, image):
        self.device = torch.device('cuda')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = PIL.Image.fromarray(image)
        image = transforms.functional.to_tensor(image).to(self.device)
        image.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
        return image[None, ...]

    def draw_joints(self, image, joints):
        count = 0
        for i in joints:
            if i==[0,0]:
                count+=1
        if count>= 3:
            return 
        for i in joints:
            cv2.circle(image, (i[0],i[1]), 2, (0,0,255), 1)
        cv2.circle(image, (joints[0][0],joints[0][1]), 2, (255,0,255), 1)
        for i in self.hand_pose['skeleton']:
            if joints[i[0]-1][0]==0 or joints[i[1]-1][0] == 0:
                break
            cv2.line(image, (joints[i[0]-1][0],joints[i[0]-1][1]), (joints[i[1]-1][0],joints[i[1]-1][1]), (0,255,0), 1)

    def execute(self, image):
        try:
            data = self.preprocess(image)
            cmap, paf = self.model_trt(data)
            cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
            counts, objects, peaks = self.parse(cmap, paf)#, cmap_threshold=0.15, link_threshold=0.15)
            joints = self.preprocessdata.joints_inference(image, counts, objects, peaks)
            self.draw_joints(image, joints)
            dist_bn_joints = self.preprocessdata.find_distance(joints)
            gesture = self.clf.predict([dist_bn_joints,[0]*self.num_parts*self.num_parts])
            gesture_joints = gesture[0]
            self.preprocessdata.prev_queue.append(gesture_joints)
            self.preprocessdata.prev_queue.pop(0)
            self.preprocessdata.print_label(image, self.preprocessdata.prev_queue, self.gesture_type)
            self.publish_image(image)
            #self.human_publish(self.get_points(counts, objects, peaks))
            rospy.loginfo("Inference complete {} hand(s) detected".format(counts[0]))
        except KeyboardInterrupt:
            rospy.loginfo("Shutting Down...")
            exit()


    def publish_image(self, image):
        img = Image()
        (img.width, img.height,n) = image.shape
        img.encoding = "rgb8"
        img.is_bigendian = 0
        img.data = image.ravel().tolist()
        img.step = 3
        self.image_pub.publish(img)

def main(args):
    rospy.init_node('rikibot_openpose', anonymous=True)
    trt_pose = RikibotPose()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main(sys.argv)
