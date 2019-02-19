#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy,cv2 , cv_bridge
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Image, CompressedImage
from rectangle_detector import RectangleDetector
from std_msgs.msg import Float32, UInt8
from find_object_2d.msg import ObjectsStamped


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

        self.result = []
        self.img_debug = None
        self.img_mask = None
        self.count_roi_debug = 0

    def cb_img_receive(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        self.img_debug = frame
        self.result, self.img_mask = self.detector(frame)
        self.count_roi_debug = 0

        self.detector.clear()

        for (img, _) in self.result:
            obj = self.bridge.cv2_to_compressed_imgmsg(img, "jpg")
            self.pub_find_obj_2d.publish(obj)

    def cb_object_find(self, msg):
        if len(self.result) == 0:
            self.img_debug = np.zeros_like(self.img_debug, dtype=np.uint8)
            self.img_mask = np.zeros_like(self.img_mask, dtype=np.uint8)
            pos1 = [0, 0]
            pos2 = [10, 10]
        else:
            _ , (pos1, pos2) = self.result[self.count_roi_debug]
            rospy.loginfo('result num: {}'.format(self.count_roi_debug) + ', result len: {}'.format(len(self.result)))

        try:
            find_obj_num = msg.objects.data[0]
            if find_obj_num == 17 or find_obj_num == 18 or find_obj_num == 19:
                self.rt_sign_count += 1
                if self.rt_sign_count >= 2:
                    rospy.loginfo("[sign] Detect Right Sign")
                    self.lt_sign_count = 0
                    self.rt_sign_count = 0
                    self.pub_detect_sign.publish(7)

                    cv2.rectangle(self.img_debug, pos1, pos2, (255, 127, 127), 1)
                    cv2.putText(self.img_debug, 'RIGHT', (pos1[0]+2, pos1[1]+2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(self.img_debug, 'w: {}, h: {}, ratio: {:.2f}'.format(pos2[0]-pos1[0], pos2[1]-pos1[1], (float(pos2[0]-pos1[0]) / (pos2[1]-pos1[1]))), (pos1[0]+2, pos1[1]+7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 127, 0), 2)
                           
            elif find_obj_num == 10 or find_obj_num == 12 or find_obj_num == 14 or find_obj_num == 15:
                self.lt_sign_count += 1
                if self.lt_sign_count >= 2:
                    rospy.loginfo("[sign] Detect Left Sign")
                    self.lt_sign_count = 0
                    self.rt_sign_count = 0
                    self.pub_detect_sign.publish(8)

                    cv2.rectangle(self.img_debug, pos1, pos2, (255, 0, 0), 1)
                    cv2.putText(self.img_debug, 'LEFT', (pos1[0]+2, pos1[1]+2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(self.img_debug, 'w: {}, h: {}, ratio: {:.2f}'.format(pos2[0]-pos1[0], pos2[1]-pos1[1], (float(pos2[0]-pos1[0]) / (pos2[1]-pos1[1]))), (pos1[0]+2, pos1[1]+7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 127, 0), 2)
                           
            else:
                rospy.loginfo("[sign] Detect Other")
                cv2.rectangle(self.img_debug, pos1, pos2, (0, 0, 255), 1)
                cv2.putText(self.img_debug, 'etc', (pos1[0]+2, pos1[1]+2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(self.img_debug, 'w: {}, h: {}, ratio: {:.2f}'.format(pos2[0]-pos1[0], pos2[1]-pos1[1], (float(pos2[0]-pos1[0]) / (pos2[1]-pos1[1]))), (pos1[0]+2, pos1[1]+7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 127, 0), 2)
                
  
        except:
            rospy.loginfo("[sign] not found callback")
            cv2.rectangle(self.img_debug, pos1, pos2, (0, 0, 127), 1)
            cv2.putText(self.img_debug, 'NONE', (pos1[0]+2, pos1[1]+2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(self.img_debug, 'w: {}, h: {}, ratio: {:.2f}'.format(pos2[0]-pos1[0], pos2[1]-pos1[1], (float(pos2[0]-pos1[0]) / (pos2[1]-pos1[1]))), (pos1[0]+2, pos1[1]+7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 127, 0), 2)

        self.count_roi_debug += 1
        if self.count_roi_debug >= len(self.result):
            self.pub_img_debug.publish(self.bridge.cv2_to_compressed_imgmsg(np.hstack((self.img_debug, self.img_mask)), "jpg"))

    @staticmethod
    def main():
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("Sign_Node")
    node = SignNode()
    node.main()
