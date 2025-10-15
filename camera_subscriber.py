#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraSaver:
    def __init__(self):
        rospy.init_node("camera_saver", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.image_callback)
        self.image_count = 0 
        rospy.loginfo("CameraSaver node started. Listening to /webcam/image_raw...")
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            filename = f"saved_image_{self.image_count}.jpg"
            cv2.imwrite(filename, cv_image)
            rospy.loginfo(f"Image saved as {filename}")
            self.image_count += 1 
        except Exception as e:
            rospy.logerr(f"Failed to save image: {e}")

if __name__ == "__main__":
    CameraSaver()
    rospy.spin()  
