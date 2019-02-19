#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import Image, CompressedImage
from mission_processor import MissionProcessor
#from tunnel_detector import TunnelDetector
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import rospy
import cv2
import time


class TunnelMissionNode:
    def __init__(self):
        # TODO : 차단바 미션 객체
        self.processor = MissionProcessor()
        #self.tunnel = TunnelDetector()
        self.cvBridge = CvBridge()

        # TODO : 미션 처리에 필요한 data
        self.img_ori = None
        self.scan_dis = [0.0] * 360
        self.odom_pos_current = [0, 0]
        self.odom_pos_start = [0, 0]
        self.detect_left_lane = False
        self.detect_right_lane = False
        self.max_intensity = 0
        self.check_time_pre = time.time()

        self.end_count = 0

        # TODO : 일반 주행을 위한 미션 노드
        self.sub_ord_rev = rospy.Subscriber('/controller/seq/tunnel', UInt8, self.cb_order_receive, queue_size=1)

        self.sub_img_rev = rospy.Subscriber('/controller/image/mission', CompressedImage, self.cb_img_receive, queue_size=1)
        self.sub_scan_dis = rospy.Subscriber('/scan', LaserScan, self.cb_scan_dis, queue_size=1)
        self.sub_odom_info = rospy.Subscriber('/odom', Odometry, self.cb_odom_info, queue_size=1)
        self.sub_left_lane = rospy.Subscriber('/driving/signal/detect_left_lane', UInt8, self.cb_left_lane, queue_size=1)
        self.sub_right_lane = rospy.Subscriber('/driving/signal/detect_right_lane', UInt8, self.cb_right_lane, queue_size=1)
        self.sub_intensity_value = rospy.Subscriber('/driving/value/max_intensity', UInt8, self.cb_max_intensity, queue_size=1)

        self.pub_driving_mode = rospy.Publisher('/mission/mod/driving', UInt8, queue_size=1)
        self.pub_angular_value = rospy.Publisher('/mission/value/angular_vel', Float32, queue_size=1)
        self.pub_linear_value = rospy.Publisher('/mission/value/linear_vel', Float32, queue_size=1)
        self.pub_shift_value = rospy.Publisher('/mission/value/error_shift', Float32, queue_size=1)
        self.pub_threshold_value = rospy.Publisher('/mission/value/lane_threshold', UInt8, queue_size=1)

        self.pub_seq_change = rospy.Publisher('/mission/seq/change', UInt8, queue_size=1)
        self.pub_delay_change = rospy.Publisher('/mission/time/delay', Float32, queue_size=1)
        self.pub_timeout_change = rospy.Publisher('/mission/time/timeout', Float32, queue_size=1)

        #self.pub_img_debug = rospy.Publisher('/mission/debug/tunnel/compressed', CompressedImage, queue_size=1)
        self.pub_img_debug = rospy.Publisher('/mission/debug/all/compressed', CompressedImage, queue_size=1)

    def fn_init_mission(self):
        self.img_ori = None
        self.scan_dis = [0.0] * 360
        self.odom_pos_current = [0, 0]
        self.odom_pos_start = [0, 0]
        self.detect_left_lane = False
        self.detect_right_lane = False
        self.max_intensity = 0
        self.check_time_pre = time.time()

    def cb_img_receive(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.img_ori = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def cb_scan_dis(self, msg):
        self.scan_dis = msg.ranges

    def cb_odom_info(self, msg):
        self.odom_pos_current = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def cb_left_lane(self, msg):
        self.detect_left_lane = bool(msg.data)

    def cb_right_lane(self, msg):
        self.detect_right_lane = bool(msg.data)

    def cb_max_intensity(self, msg):
        self.max_intensity = msg.data

    def cb_order_receive(self, msg):
        #process_time_pre = time.time()
        driving_state = 'none'
        mission_time_delay = 0.1
        mission_timeout = 0.0
        info_msg = ''

        if msg.data == 90:

            # TODO : 이미지 및 미션 처리

            # TODO : 미션 판단
            if True:
                rospy.loginfo('Order : [seq.pre] Tunnel sign detect')
                self.pub_seq_change.publish(94)

            info_msg = '? : '

        elif msg.data == 99:
            # TODO : 초기화
            rospy.loginfo('Order : [seq.init] init mission')
            self.fn_init_mission()

        elif msg.data == 1:
            # TODO : 주행 모드 설정
            driving_state = 'lane following in bright lane'

            # TODO : 이미지 및 미션 처리
            scan_count = self.processor.fn_cal_scan_count(self.scan_dis[240:290], 0.3)

            # TODO : 미션 판단
            if (scan_count > 30) and (self.max_intensity < 180):
                rospy.loginfo('Order : [seq.1] Get in tunnel')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(2)

            info_msg = 'scan count: {}'.format(scan_count) + ', max intensity: {}'.format(self.max_intensity)

        elif msg.data == 2:
            # TODO : 주행 모드 설정
            driving_state = 'stop'
            mission_time_delay = 0.02

            # TODO : 이미지 및 미션 처리
            check_time = time.time() - self.check_time_pre

            # TODO : 미션 판단
            if (check_time) > 0.05:
                rospy.loginfo('Order : [seq.2] Tunnel Start')
                self.odom_pos_start = self.odom_pos_current
                self.pub_seq_change.publish(3)

            # TODO : 디버깅
            info_msg = 'check time: {}'.format(check_time)

        elif msg.data == 3:
            # TODO : 주행 모드 설정
            driving_state = 'tunnel'

            # TODO : 이미지 및 미션 처리
            odom_dis = self.processor.fn_cal_odom_dis(self.odom_pos_start, self.odom_pos_current)

            # TODO : 미션 판단
            if self.detect_left_lane and self.detect_right_lane and (self.max_intensity > 180) and (odom_dis > 1.8):
                rospy.loginfo('Order : [seq.3] Tunnel End')
                self.pub_threshold_value.publish(240)
                self.pub_seq_change.publish(4)

            # TODO : 디버깅
            info_msg = 'detect lane: {} {}'.format(self.detect_left_lane, self.detect_right_lane) + ', max intensity: {}'.format(self.max_intensity) + ', odom dis: {0:.6f}'.format(odom_dis)

        elif msg.data == 4:
            # TODO : flag 초기화
            driving_state = 'lane following'
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
            rospy.loginfo('[tunnel][seq: ' + str(msg.data) + ']' + '\n>> ' + info_msg)

    def fn_driving_mode(self, driving_state):
        if driving_state == 'lane following':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_shift_value.publish(0.0) #미션별로 값주기 (pixel)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.lane_mode.value)

        elif driving_state == 'lane following in bright lane':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_shift_value.publish(0.0) #미션별로 값주기 (pixel)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.intensity_lane_mode.value)

        elif driving_state == 'go straight':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(0.0) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'stop':
            self.pub_linear_value.publish(0.0) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(0.0) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'tunnel':
            self.pub_driving_mode.publish(self.processor.driving_mode_step.tunnel_mode.value)

        elif driving_state == 'none':
            #rospy.loginfo('입력되지않은 주행 모드')
            pass

        else:
            rospy.logerr('잘못된 주행 모드')

    @staticmethod
    def main():
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('Tunnel_Mission_Node')
    node = TunnelMissionNode()
    node.main()

