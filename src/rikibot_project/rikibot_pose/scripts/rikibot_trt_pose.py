#! /usr/bin/env python3
import rospy
import os
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
import PIL.Image
from rikibot_pose.msg import *


class RikibotPose:
    def __init__(self):

        self.image_pub = rospy.Publisher("/rikibot_pose/img_np",Image,queue_size=1)
        self.human_pub = rospy.Publisher("/rikibot_pose/humans", Human,queue_size=1)
        self.human_msg = Human()

        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = dir_path.replace('rikibot_pose/scripts', 'rikibot_pose/')

        self.model = dir_path + 'models/trt_resnet18_baseline_att_224x224_A_epoch_249.pth'
        self.pose_config = dir_path +'models/human_pose.json'

        with open(self.pose_config, 'r') as f:
            self.human_pose = json.load(f)

        self.topology = trt_pose.coco.coco_category_to_topology(self.human_pose)
        self.num_parts = len(self.human_pose['keypoints'])
        self.num_links = len(self.human_pose['skeleton'])
        self.model_trt = TRTModule()
        rospy.loginfo("load model")
        self.model_trt.load_state_dict(torch.load(self.model))
        self.mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
        self.std = torch.Tensor([0.229, 0.224, 0.225]).cuda()
        self.device = torch.device('cuda')
        rospy.loginfo("load model done")

        self.parse = ParseObjects(self.topology)
        self.draw = DrawObjects(self.topology)

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

    def execute(self, image):
        try:
            data = self.preprocess(image)
            cmap, paf = self.model_trt(data)
            cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
            counts, objects, peaks = self.parse(cmap, paf)#, cmap_threshold=0.15, link_threshold=0.15)
            self.draw(image, counts, objects, peaks)
            self.publish_image(image)
            self.human_publish(self.get_points(counts, objects, peaks))
            rospy.loginfo("Inference complete {} person(s) detected".format(counts[0]))
        except KeyboardInterrupt:
            rospy.loginfo("Shutting Down...")
            exit()

    def get_points(self, counts, objects, peaks):
        height = 224
        width = 224
        k = self.topology.shape[0]
        count = counts[0]
        people = []
        for human in range(count):
            obj = objects[0][human]
            person = {}
            for key in range(obj.shape[0]):
                value = int(obj[key])
                if value >=0:
                    peak = peaks[0][key][value]
                    x,y = (round(float(peak[1])*width),round(float(peak[0])*height))
                else:
                    x,y = -1,-1
                person[self.human_pose['keypoints'][key]]=(x,y)
            people.append(person)
        return people

    def publish_image(self, image):
        img = Image()
        (img.width, img.height,n) = image.shape
        img.encoding = "rgb8"
        img.is_bigendian = 0
        img.data = image.ravel().tolist()
        img.step = 3
        self.image_pub.publish(img)

    def human_publish(self, people):
        for i in range(len(people)):
            self.human_msg.id = i
            for key in people[i]:
                part = keypoint()
                part.x,part.y = people[i][key]
                setattr(self.human_msg,key,part)
            self.human_pub.publish(self.human_msg)
 
def main(args):
    rospy.init_node('rikibot_openpose', anonymous=True)
    trt_pose = RikibotPose()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main(sys.argv)
