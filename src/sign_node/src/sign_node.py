#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, cv2, cv_bridge, time
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
        #self.pub_find_obj_2d = rospy.Publisher('/sign/image/find_object_2d', Image, queue_size=10)
        self.sub_obj_stamped = rospy.Subscriber('objectsStamped', ObjectsStamped, self.cb_object_find, queue_size=1)

        self.pub_detect_sign = rospy.Publisher('/sign/signal/detect_sign', UInt8, queue_size=1)

        self.pub_img_debug = rospy.Publisher('/sign/debug/object/compressed', CompressedImage, queue_size=1)

        self.dict_color = {'left':(255, 0, 0), 'right':(255, 255, 0), 'etc':(0, 0, 127), 'none':(0, 0, 0), 'time over':(0, 0, 255)}

        self.lt_sign_count = 0
        self.rt_sign_count = 0
        self.process_time_pre = time.time()

        self.list_result = []
        self.img_debug = None
        self.img_mask = None
        self.list_detect_object = []

        self.flag_publish_roi = False
        self.count_publish_roi = 0
        self.publish_time_pre = time.time()

        loop_rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.fn_publish_roi()
            loop_rate.sleep()

    def fn_debug_detect(self):
        if self.img_debug is None or self.img_mask is None:
            return

        if len(self.list_detect_object) > len(self.list_result):
            self.list_detect_object = self.list_detect_object[:len(self.list_result)]

        for ii in range(len(self.list_result)):
            _ , (pos1, pos2) = self.list_result[ii]
            val_width = pos2[0] - pos1[0]
            val_height = pos2[1] - pos1[1]

            if ii < len(self.list_detect_object):
                detect_object = self.list_detect_object[ii]
            else:
                detect_object = 'time over'

            cv2.rectangle(self.img_debug, pos1, pos2, self.dict_color[detect_object], 3)
            cv2.putText(self.img_debug, detect_object, (pos1[0]+3, pos1[1]+5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(self.img_debug, 'w: {}, h: {}, ratio: {:.2f}'.format(val_width, val_height, (float(val_width) / (val_height))), (pos1[0]+3, pos1[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 127, 0), 2)

        self.pub_img_debug.publish(self.bridge.cv2_to_compressed_imgmsg(np.hstack((self.img_debug, self.img_mask)), "jpg"))

    def fn_publish_roi(self):
        if self.flag_publish_roi:
            try:
                if self.count_publish_roi >= len(self.list_result):
                    return
                (img, _) = self.list_result[self.count_publish_roi]
                obj = self.bridge.cv2_to_compressed_imgmsg(img, "jpg")
                #obj = self.bridge.cv2_to_imgmsg(img, "bgr8")
                self.pub_find_obj_2d.publish(obj)
                self.count_publish_roi += 1
                self.flag_publish_roi = False
            except Exception as e:
                rospy.logerr('Publish process over : {}'.format(e))

        else:
            if self.count_publish_roi == 0:
                publish_time = time.time() - self.publish_time_pre
                if publish_time > 0.03:
                    #rospy.logwarn('메시지 수신을 무시했습니다.')
                    self.flag_publish_roi = True

    def cb_img_receive(self, msg):
        self.process_time_pre = time.time()
        self.fn_debug_detect()

        np_arr = np.fromstring(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = frame[:240, :].copy()

        self.list_detect_object = []
        self.img_debug = frame
        self.list_result, self.img_mask = self.detector(frame)

        self.detector.clear()

        self.count_publish_roi = 0
        self.publish_time_pre = time.time()

    def cb_object_find(self, msg):
        try:
            find_obj_num = msg.objects.data[0]
            if find_obj_num == 24 or find_obj_num == 25 or find_obj_num == 26 or find_obj_num == 27:
                self.rt_sign_count += 1
                if self.rt_sign_count >= 2:
                    rospy.loginfo("[sign] Detect Right Sign")
                    self.lt_sign_count = 0
                    self.rt_sign_count = 0
                    self.pub_detect_sign.publish(7)
                self.list_detect_object.append('right')

            elif find_obj_num == 20 or find_obj_num == 21 or find_obj_num == 22 or find_obj_num == 23:
                self.lt_sign_count += 1
                if self.lt_sign_count >= 2:
                    rospy.loginfo("[sign] Detect Left Sign")
                    self.lt_sign_count = 0
                    self.rt_sign_count = 0
                    self.pub_detect_sign.publish(8)
                self.list_detect_object.append('left')
            else:
                #rospy.loginfo("[sign] Detect Other")
                self.list_detect_object.append('etc')
  
        except:
            #rospy.loginfo("[sign] not found callback")
            self.list_detect_object.append('none')

        process_time = time.time() - self.process_time_pre
        self.process_time_pre = time.time()
        #if len(self.list_detect_object) == len(self.list_result):
            #rospy.loginfo('process time (all scene) [{0}, {1}] : {2:.6f}'.format(len(self.list_detect_object), len(self.list_result), process_time))
        #elif len(self.list_detect_object) < len(self.list_result):
            #rospy.loginfo('process time (1 scene) [{0}, {1}] : {2:.6f}'.format(len(self.list_detect_object), len(self.list_result), process_time))
        #else:
            #rospy.loginfo('process time (over scene) [{0}, {1}] : {2:.6f}'.format(len(self.list_detect_object), len(self.list_result), process_time))
        self.flag_publish_roi = True

    @staticmethod
    def main():
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("Sign_Node")
    node = SignNode()
    node.main()

np.zeros_like()
