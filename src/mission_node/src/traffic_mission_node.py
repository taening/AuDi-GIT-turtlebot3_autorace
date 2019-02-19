#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import Image, CompressedImage
from mission_processor import MissionProcessor
from traffic_detector import TrafficDetector
from cv_bridge import CvBridge
import numpy as np
import rospy
import time
import cv2


class TrafficMissionNode:
    def __init__(self):
        # TODO : 미션 처리를 위한 객체
        self.processor = MissionProcessor()
        self.traffic = TrafficDetector()
        self.cvBridge = CvBridge()

        # TODO : 미션 처리에 필요한 data
        self.img_ori = None
        self.check_time_pre = time.time()

        self.end_count = 0

        # TODO : 주행을 위한 퍼블리셔 및 서브스크라이버
        self.sub_ord_rev = rospy.Subscriber('/controller/seq/traffic', UInt8, self.cb_order_receive, queue_size=1)

        self.sub_img_rev = rospy.Subscriber('/controller/image/mission', CompressedImage, self.cb_img_receive, queue_size=1)

        self.pub_driving_mode = rospy.Publisher('/mission/mod/driving', UInt8, queue_size=1)
        self.pub_angular_value = rospy.Publisher('/mission/value/angular_vel', Float32, queue_size=1)
        self.pub_linear_value = rospy.Publisher('/mission/value/linear_vel', Float32, queue_size=1)
        self.pub_shift_value = rospy.Publisher('/mission/value/error_shift', Float32, queue_size=1)

        self.pub_seq_change = rospy.Publisher('/mission/seq/change', UInt8, queue_size=1)
        self.pub_delay_change = rospy.Publisher('/mission/time/delay', Float32, queue_size=1)
        self.pub_timeout_change = rospy.Publisher('/mission/time/timeout', Float32, queue_size=1)

        #self.pub_img_debug = rospy.Publisher('/mission/debug/traffic/compressed', CompressedImage, queue_size=1)
        self.pub_img_debug = rospy.Publisher('/mission/debug/all/compressed', CompressedImage, queue_size=1)

    def fn_init_mission(self):
        self.traffic.fn_init_red()
        self.traffic.fn_init_yellow()
        self.traffic.fn_init_green()
        self.traffic.fn_init_light()
        self.img_ori = None
        self.check_time_pre = time.time()

    def cb_img_receive(self, msg):
        # TODO : 이미지 저장
        np_arr = np.fromstring(msg.data, np.uint8)
        self.img_ori = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def cb_order_receive(self, msg):
        driving_state = 'none'
        mission_time_delay = 0.1
        mission_timeout = 0.0
        info_msg = ''

        if msg.data == 90:
            pass
            # TODO : 이미지 및 미션 처리

            # TODO : 미션 판단

            # TODO : 디버깅

        elif msg.data == 99:
            # TODO : 초기화
            rospy.loginfo('Order : [seq.init] init mission')
            self.fn_init_mission()

        elif msg.data == 1:
            # TODO : 주행 모드 설정
            driving_state = 'stop'

            # TODO : 이미지 및 미션 처리
            (count_red, _, count_green), img_debug = self.traffic.fn_traffic_count_fixed_light(self.img_ori)
            #count_red, img_debug_r = self.traffic.fn_traffic_count_red(self.img_ori)
            #count_green, img_debug_g = self.traffic.fn_traffic_count_green(self.img_ori)
            #img_debug = cv2.bitwise_or(img_debug_r, img_debug_g)

            # TODO : 미션 판단
            if count_red >= 5:
                rospy.loginfo('Order : [seq.1A] red light detect')
                self.check_time_pre = time.time()
                self.traffic.fn_init_green()
                self.pub_seq_change.publish(2)
            elif count_green >= 2:
                rospy.loginfo('Order : [seq.1B] green light detect, go!')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(3)

            # TODO : 디버깅
            info_msg = 'red traffic count: {}'.format(count_red) + ', green traffic count: {}'.format(count_green)
            self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_debug, "jpg"))

        elif msg.data == 2:
            # TODO : 주행 모드 설정
            driving_state = 'stop'

            # TODO : 이미지 및 미션 처리
            (_, _, count_green), img_debug = self.traffic.fn_traffic_count_fixed_light(self.img_ori)
            #count_green, img_debug = self.traffic.fn_traffic_count_green_in_red(self.img_ori, 5)

            # TODO : 미션 판단
            if count_green >= 2:
                rospy.loginfo('Order : [seq.2] green light detect, go!')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(3)

            # TODO : 디버깅
            info_msg = 'green traffic count: {}'.format(count_green)
            self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_debug, "jpg"))

        elif msg.data == 3:
            # TODO : 주행 모드 설정
            driving_state = 'go straight'

            # TODO : 이미지 및 미션 처리
            check_time = time.time() - self.check_time_pre

            # TODO : 미션 판단
            if check_time >= 2.0:
                rospy.loginfo('Order : [seq.3] exit traffic')
                self.pub_seq_change.publish(4)

            # TODO : 디버깅
            info_msg = 'check time: {:.4f}'.format(check_time)

        elif msg.data == 4:
            # TODO : 주행 모드 설정
            driving_state = 'lane following'

            # TODO : flag 초기화
            self.fn_init_mission()

            # TODO : 종료 카운트
            if self.end_count >= 5:
                self.end_count = 0
                rospy.loginfo('Order : [seq.4] mission success !!')
                self.pub_seq_change.publish(100) # 100 : mission end point
            else:
                self.end_count += 1

            # TODO : 디버깅
            info_msg = 'end count: {}'.format(self.end_count)

        self.fn_driving_mode(driving_state)
        if msg.data < 90:
            self.pub_delay_change.publish(mission_time_delay)
            self.pub_timeout_change.publish(mission_timeout)
            rospy.loginfo('[traffic][seq: ' + str(msg.data) + ']' + '\n>> ' + info_msg)

    def fn_driving_mode(self, driving_state):
        # TODO : 주행 모드 별 동작
        if driving_state == 'lane following':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_shift_value.publish(0.0) #미션별로 값주기 (pixel)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.lane_mode.value)

        elif driving_state == 'go straight':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(0.0) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'stop':
            self.pub_linear_value.publish(0.0) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(0.0) #미션별로 값주기 (rad/s)
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
    rospy.init_node('Traffic_Mission_Node')
    node = TrafficMissionNode()
    node.main()

