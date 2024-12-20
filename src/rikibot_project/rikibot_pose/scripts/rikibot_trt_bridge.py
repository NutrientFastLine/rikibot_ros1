#! /usr/bin/env python

import rospy
import numpy as np
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import ImageFont, ImageDraw
from PIL import Image as IMG

class RikibotTrtBridge:
    def __init__(self):
        rospy.Subscriber('/rikibot_pose/img_np',Image, self.np2img)
        self.image_pub = rospy.Publisher('/rikibot_pose/image_raw',Image,queue_size=1)
        self.bridge = CvBridge()

    def np2img(self, msg):
        img = np.array(list(bytearray(msg.data)),dtype='uint8')
        img = img.reshape(msg.width,msg.height,3)
        cv2_im_rgb = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img,"bgr8"))
            #rospy.loginfo("Inference image published")
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('rikibot_trt_bridge', anonymous=True)
    trt_bridge = RikibotTrtBridge()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main(sys.argv)

