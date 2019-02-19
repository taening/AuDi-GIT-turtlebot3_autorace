#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy,cv2 , cv_bridge
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Image, CompressedImage
from rectangle_detector import RectangleDetector
from std_msgs.msg import Float32, UInt8
from find_object_2d.msg import ObjectsStamped
import time


class SignNode:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.detector = RectangleDetector()

        self.sub_sign_img = rospy.Subscriber('/controller/image/sign', CompressedImage, self.cb_img_receive)

        self.pub_find_obj_2d = rospy.Publisher('/sign/image/find_object_2d/compressed', CompressedImage, queue_size=1)
        self.sub_obj_stamped = rospy.Subscriber('objectsStamped', ObjectsStamped, self.cb_object_find, queue_size=1)

        self.pub_detect_sign = rospy.Publisher('/sign/signal/detect_sign', UInt8, queue_size=1)

        self.pub_img_debug = rospy.Publisher('/sign/debug/object/compressed', Image, queue_size=1)
        self.lt_sign_count = 0
        self.rt_sign_count = 0
        self.time_pre = time.time()

    def cb_img_receive(self, msg):
        self.time_pre = time.time()
        np_arr = np.fromstring(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        result = self.detector(frame)
        self.detector.clear()

        for img in result:
            obj = self.bridge.cv2_to_compressed_imgmsg(img, "jpg")
            self.pub_find_obj_2d.publish(obj)

    def cb_object_find(self, msg):
        try:
            find_obj_num = msg.objects.data[0]
            if find_obj_num == 17 or find_obj_num == 18 or find_obj_num == 19:
     
                self.rt_sign_count += 1
                if self.rt_sign_count >= 2:
                    rospy.loginfo("[sign] Detect Right Sign")
                    self.lt_sign_count = 0
                    self.rt_sign_count = 0
                    self.pub_detect_sign.publish(7)
                           
            elif find_obj_num == 10 or find_obj_num == 12 or find_obj_num == 14 or find_obj_num == 15:

                self.lt_sign_count += 1
                if self.lt_sign_count >= 2:
                    rospy.loginfo("[sign] Detect Left Sign")
                    self.lt_sign_count = 0
                    self.rt_sign_count = 0
                    self.pub_detect_sign.publish(8)
                           
            else:
                pass
                
  
        except:
            rospy.loginfo("[sign] not found callback")

        rospy.loginfo('time: {0:.6}'.format(time.time() - self.time_pre))

    @staticmethod
    def main():
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("Sign_Node")
    node = SignNode()
    node.main()
