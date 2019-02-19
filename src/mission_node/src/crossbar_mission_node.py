#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import Image, CompressedImage
from mission_processor import MissionProcessor
from crossbar_detector import CrossbarDetector
from cv_bridge import CvBridge
import numpy as np
import rospy
import cv2
import time


class CrossbarMissionNode:
    def __init__(self):
        # TODO : 미션 처리를 위한 객체
        self.processor = MissionProcessor()
        self.crossbar = CrossbarDetector()
        self.cvBridge = CvBridge()

        # TODO : 미션 처리에 필요한 data
        self.img_ori = None
        self.ultrasonic_dis = [0.0] * 5
        self.crossbar_count = 0
        self.crossbar_not_count = 0
        self.check_time_pre = time.time()

        self.end_count = 0

        # TODO : 주행을 위한 퍼블리셔 및 서브스크라이버
        self.sub_ord_rev = rospy.Subscriber('/controller/seq/crossbar', UInt8, self.cb_order_receive, queue_size=1)

        self.sub_img_rev = rospy.Subscriber('/controller/image/mission', CompressedImage, self.cb_img_receive, queue_size=1)
        self.sub_ultrasonic_dis = rospy.Subscriber('/ultrasonic_distance', Float32, self.cb_ultrasonic_dis, queue_size=1)

        self.pub_driving_mode = rospy.Publisher('/mission/mod/driving', UInt8, queue_size=1)
        self.pub_angular_value = rospy.Publisher('/mission/value/angular_vel', Float32, queue_size=1)
        self.pub_linear_value = rospy.Publisher('/mission/value/linear_vel', Float32, queue_size=1)
        self.pub_shift_value = rospy.Publisher('/mission/value/error_shift', Float32, queue_size=1)

        self.pub_seq_change = rospy.Publisher('/mission/seq/change', UInt8, queue_size=1)
        self.pub_delay_change = rospy.Publisher('/mission/time/delay', Float32, queue_size=1)
        self.pub_timeout_change = rospy.Publisher('/mission/time/timeout', Float32, queue_size=1)

        #self.pub_img_debug = rospy.Publisher('/mission/debug/crossbar/compressed', CompressedImage, queue_size=1)
        self.pub_img_debug = rospy.Publisher('/mission/debug/all/compressed', CompressedImage, queue_size=1)

    def fn_init_mission(self):
        self.img_ori = None
        self.ultrasonic_dis = [0.0] * 5
        self.crossbar_count = 0
        self.crossbar_not_count = 0
        self.check_time_pre = time.time()

    def cb_img_receive(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.img_ori = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def cb_ultrasonic_dis(self, msg):
        self.ultrasonic_dis = self.ultrasonic_dis[-4:] + [msg.data]

    def cb_order_receive(self, msg):
        driving_state = 'none'
        mission_time_delay = 0.1
        mission_timeout = 0.0
        info_msg = ''

        if msg.data == 90:
            # TODO : 이미지 및 미션 처리
            #flag_crossbar, img_crossbar = self.crossbar.fn_find_crossbar(self.img_ori)

            # TODO : 미션 판단
            #if flag_crossbar:
            if True:
                rospy.loginfo('Order : [seq.pre] Crossbar detect')
                self.pub_seq_change.publish(93)

            # TODO : 디버깅
            #info_msg = 'flag of crossbar : {}'.format(flag_crossbar)
            #self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_crossbar, "jpg"))

        elif msg.data == 99:
            # TODO : 초기화
            rospy.loginfo('Order : [seq.init] init mission')
            self.fn_init_mission()

        elif msg.data == 1:
            # TODO : 주행 모드 설정
            driving_state = 'lane following'
            mission_time_delay = 0.04

            # TODO : 이미지 및 미션 처리
            _, img_crossbar = self.crossbar.fn_confirm_crossbar(self.img_ori)
            mean_dis = self.processor.fn_cal_ultrasonic_dis(self.ultrasonic_dis)

            # TODO : 미션 판단
            if 0.0 < mean_dis < 22.0:
                rospy.loginfo('Order : [seq.1] Ultrasonic distance is satisfied')
                self.crossbar_count = 0
                self.crossbar_not_count = 0
                self.pub_seq_change.publish(2)

            # TODO : 디버깅
            info_msg = 'ultrasonic: {}'.format(mean_dis)
            self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_crossbar, "jpg"))

        elif msg.data == 2:
            # TODO : 주행 모드 설정
            driving_state = 'stop'

            # TODO : 이미지 및 미션 처리
            flag_crossbar, img_crossbar = self.crossbar.fn_confirm_crossbar(self.img_ori)
            mean_dis = self.processor.fn_cal_ultrasonic_dis(self.ultrasonic_dis)

            # TODO : 미션 판단
            if flag_crossbar and (0.0 < mean_dis < 22.0):
                self.crossbar_count += 1
                self.crossbar_not_count = 0
            elif not flag_crossbar:
                self.crossbar_not_count += 1
                self.crossbar_count = 0
            else:
                rospy.loginfo('Order : [seq.2C] Ultrasonic distance is not satisfied')
                self.end_count = 0
                self.pub_seq_change.publish(1)

            if self.crossbar_count >= 3:
                rospy.loginfo('Order : [seq.2A] Confirm crossbar')
                self.pub_seq_change.publish(3)
            elif self.crossbar_not_count >= 6:
                rospy.loginfo('Order : [seq.2B] Don\'t confirm crossbar')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(4)

            # TODO : 디버깅
            info_msg = ', ultrasonic: {}'.format(mean_dis) + ', flag of crossbar: {}'.format(flag_crossbar) + ', count: {}'.format(self.crossbar_count) + ', not count: {}'.format(self.crossbar_not_count)
            self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_crossbar, "jpg"))

        elif msg.data == 3:
            # TODO : 주행 모드 설정
            driving_state = 'stop'
            mission_time_delay = 0.03

            # TODO : 이미지 및 미션 처리
            flag_crossbar, img_crossbar = self.crossbar.fn_confirm_crossbar(self.img_ori)
            mean_dis = self.processor.fn_cal_ultrasonic_dis(self.ultrasonic_dis)

            # TODO : 미션 판단
            if not (0.0 < mean_dis < 22.0):
                rospy.loginfo('Order : [seq.3] Crossbar not detect, more')
                self.end_count = 0
                self.pub_seq_change.publish(5)

            # TODO : 디버깅
            info_msg = 'flag of crossbar: {}'.format(flag_crossbar) + ', ultrasonic: {}'.format(mean_dis)
            self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_crossbar, "jpg"))

        elif msg.data == 4:
            # TODO : 주행 모드 설정
            driving_state = 'lane following'

            # TODO : 이미지 및 미션 처리
            check_time = time.time() - self.check_time_pre

            # TODO : 미션 판단
            if check_time > 1.0:
                rospy.loginfo('Order : [seq.4] Restart detecting crossbar')
                self.pub_seq_change.publish(1)

            # TODO : 디버깅
            info_msg = 'check time: {0:.6f}'.format(check_time)

        elif msg.data == 5:
            # TODO : flag 초기화
            driving_state = 'lane following'
            self.fn_init_mission()

            # TODO : 종료 카운트
            if self.end_count >= 5:
                self.end_count = 0
                rospy.loginfo('Order : [seq.5] mission success !!')
                self.pub_seq_change.publish(100) # 100 : mission end point
            else:
                self.end_count += 1

            # TODO : 디버깅
            info_msg = 'end count: {}'.format(self.end_count)

        self.fn_driving_mode(driving_state)
        if msg.data < 90:
            self.pub_delay_change.publish(mission_time_delay)
            self.pub_timeout_change.publish(mission_timeout)
            rospy.loginfo('[crossbar][seq: ' + str(msg.data) + ']' + '\n>> ' + info_msg)

    def fn_driving_mode(self, driving_state):
        # TODO : 주행 모드 별 동작
        if driving_state == 'lane following':
            self.pub_linear_value.publish(0.22)
            self.pub_shift_value.publish(0.0)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.lane_mode.value)

        elif driving_state == 'stop':
            self.pub_linear_value.publish(0.0)
            self.pub_angular_value.publish(0.0)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'none':
            #rospy.loginfo('입력되지않은 주행 모드')
            pass

        else:
            rospy.logerr('잘못된 주행 모드')

    @staticmethod
    def main():
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('Crossbar_Mission_Node')
    node = CrossbarMissionNode()
    node.main()

