import rospy
import os,cv2
import keyboard
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CompressedImage

class FaceLocation():
    def __init__(self):
        rospy.init_node('riki_face_tracker_node', log_level=rospy.INFO)
        self.rate = rospy.get_param('~rate', 10)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw/')
        r = rospy.Rate(self.rate)
        self.name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)

        self.sub_image_type = "raw"
        self.counter = 1
        
 	self.dir_path = os.path.dirname(os.path.realpath(__file__))

	self.dir_path = self.dir_path.replace('riki_autorace_detect/nodes', 'riki_autorace_detect/')
        self.dir_path += 'image/'


        if self.sub_image_type == "compressed":
            self.sub_image_original = rospy.Subscriber(self.camera_topic+'compressed', CompressedImage, self.ImageCallback, queue_size = 1)
        elif self.sub_image_type == "raw":
            self.sub_image_original = rospy.Subscriber(self.camera_topic, Image, self.ImageCallback, queue_size = 1)

        self.cvBridge = CvBridge()
          
        while not rospy.is_shutdown():
            r.sleep()

    def ImageCallback(self, image_msg):
        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        #cv2.imshow('image',cv_image)
        if keyboard.is_pressed('s'): 
            rospy.loginfo("save image")
            cv2.imwrite(self.dir_path+str(self.counter) + ".JPG", cv_image)
            self.counter += 1


    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown face_location_node ...")

if __name__ == '__main__':
    try:
        face_location = FaceLocation()
        rospy.spin()
    except Exception, e:
        rospy.logerr("%s", str(e))
        os._exit(1)
