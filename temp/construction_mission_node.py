#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import Image, CompressedImage
from mission_processor import MissionProcessor
from construction_detector import ConstructionDetector
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
import numpy as np
import rospy
import cv2
import time
import math


class ConstructionMissionNode:
    def __init__(self):
        # TODO : 주차 미션 객체
        self.processor = MissionProcessor()
        self.construction = ConstructionDetector()
        self.cvBridge = CvBridge()

        # TODO : 미션 처리에 필요한 data
        self.img_ori = None
        self.scan_dis = [0.0] * 360
        self.ultrasonic_dis = [0.0] * 3
        self.check_time_pre = time.time()

        self.end_count = 0

        # TODO : 일반 주행을 위한 미션 노드
        self.sub_ord_rev = rospy.Subscriber('/controller/seq/construction', UInt8, self.cb_order_receive, queue_size=1)

        self.sub_img_rev = rospy.Subscriber('/controller/image/mission', CompressedImage, self.cb_img_receive, queue_size=1)
        self.sub_scan_dis = rospy.Subscriber('/scan', LaserScan, self.cb_scan_dis, queue_size=1)
        self.sub_ultrasonic_dis = rospy.Subscriber('/ultrasonic_distance', Float32, self.cb_ultrasonic_dis, queue_size=1)

        self.pub_driving_mode = rospy.Publisher('/mission/mod/driving', UInt8, queue_size=1)
        self.pub_crossroad_mode = rospy.Publisher('/mission/mod/crossroad', UInt8, queue_size=1)
        self.pub_angular_value = rospy.Publisher('/mission/value/angular_vel', Float32, queue_size=1)
        self.pub_linear_value = rospy.Publisher('/mission/value/linear_vel', Float32, queue_size=1)
        self.pub_shift_value = rospy.Publisher('/mission/value/error_shift', Float32, queue_size=1)

        self.pub_seq_change = rospy.Publisher('/mission/seq/change', UInt8, queue_size=1)
        self.pub_delay_change = rospy.Publisher('/mission/time/delay', Float32, queue_size=1)
        self.pub_timeout_change = rospy.Publisher('/mission/time/timeout', Float32, queue_size=1)

        self.pub_img_debug = rospy.Publisher('/mission/debug/construction', Image, queue_size=1)

    def fn_init_mission(self):
        self.img_ori = None
        self.scan_dis = [0.0] * 360
        self.ultrasonic_dis = [0.0] * 3
        self.check_time_pre = time.time()

    def cb_img_receive(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.img_ori = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def cb_scan_dis(self, msg):
        self.scan_dis = msg.ranges

    def cb_ultrasonic_dis(self, msg):
        self.ultrasonic_dis = self.ultrasonic_dis[-2:] + [msg.data]

    def cb_order_receive(self, msg):
        process_time_pre = time.time()
        driving_state = 'none'
        mission_time_delay = 0.1
        mission_timeout = 0.0
        info_msg = ''

        if msg.data == 90:
            # TODO : 이미지 및 미션 처리
            forward_obstacle_count = self.processor.fn_cal_scan_count(self.scan_dis[330:359]+self.scan_dis[0: 30], 0.50) # 앞쪽 장애물 여부 판단
            left_obstacle_count = self.processor.fn_cal_scan_count(self.scan_dis[60:100], 0.30) # 왼쪽 장애물 여부 판단

            # TODO : 미션 판단
            if 15 < forward_obstacle_count < 25 and 13 < left_obstacle_count < 25:
                rospy.loginfo('Order : [seq.pre] Construction Sign detect')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(96)

            info_msg = 'forward: ' + str(forward_obstacle_count) + 'left: ' + str(left_obstacle_count)

        elif msg.data == 1:
            # TODO : ?? ?? ??
            driving_state = 'right lane following'

            # TODO : ??? ? ?? ??
            ultrasonic_mean_dis = self.processor.fn_cal_ultrasonic_dis(self.ultrasonic_dis)

            # TODO : ?? ??
            if 0 < ultrasonic_mean_dis < 35:
                rospy.loginfo('Order : [seq.1] Detect 1st construction')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(2)
                driving_state = 'left move'

            info_msg = 'ultrasonic: ' + str(ultrasonic_mean_dis)

        elif msg.data == 2:
            # TODO : 주행 모드 설정
            driving_state = 'left move'
            mission_time_delay = 0.01

            # TODO : 이미지 및 미션 처리
            check_time_now = time.time()

            # TODO : 미션 판단
            if (check_time_now - self.check_time_pre) > 0.6:
                rospy.loginfo('Order : [seq.2] Avoid 1st construction')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(3)
                driving_state = 'go straight'

            info_msg = 'check time: ' + str(check_time_now - self.check_time_pre)

        elif msg.data == 3:
            # TODO : 주행 모드 설정
            driving_state = 'go straight'
            mission_time_delay = 0.01

            # TODO : 이미지 및 미션 처리
            check_time_now = time.time()

            # TODO : 미션 판단
            if (check_time_now - self.check_time_pre) > 1.55:
                rospy.loginfo('Order : [seq.3] Exit 1st construction')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(4)
                driving_state = 'right move'

            info_msg = 'check time: ' + str(check_time_now - self.check_time_pre)

        elif msg.data == 4:
            # TODO : 주행 모드 설정
            driving_state = 'right move'
            mission_time_delay = 0.01

            # TODO : 이미지 및 미션 처리
            check_time_now = time.time()

            # TODO : 미션 판단
            if (check_time_now - self.check_time_pre) > 0.6:
                rospy.loginfo('Order : [seq.4] Normal driving')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(5)
                driving_state = 'go straight'

            info_msg = 'check time: ' + str(check_time_now - self.check_time_pre)

        elif msg.data == 5:
            # TODO : 주행 모드 설정
            driving_state = 'go straight'

            # TODO : 이미지 및 미션 처리
            check_time_now = time.time()
            right_obstacle_count = self.processor.fn_cal_scan_count(self.scan_dis[240:275], 0.30) # 오른쪽 장애물 여부 판단

            # TODO : 미션 판단
            if (check_time_now - self.check_time_pre) > 0.5 and 6 < right_obstacle_count:
                rospy.loginfo('Order : [seq.5] Detect 2nd construction')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(6)
                driving_state = 'right move'

            info_msg = 'right obstacle count: ' + str(right_obstacle_count)

        elif msg.data == 6:
            # TODO : 주행 모드 설정
            driving_state = 'right move'
            mission_time_delay = 0.01

            # TODO : 이미지 및 미션 처리
            check_time_now = time.time()

            # TODO : 미션 판단
            if (check_time_now - self.check_time_pre) > 0.65:
                rospy.loginfo('Order : [seq.6] Avoid 2nd construction')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(7)
                driving_state = 'go straight'

            info_msg = 'check time: ' + str(check_time_now - self.check_time_pre)

        elif msg.data == 7:
            # TODO : 주행 모드 설정
            driving_state = 'go straight'
            mission_time_delay = 0.01

            # TODO : 이미지 및 미션 처리
            check_time_now = time.time()

            # TODO : 미션 판단
            if (check_time_now - self.check_time_pre) > 1.0:
                rospy.loginfo('Order : [seq.7] Exit 2nd construction')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(8)
                driving_state = 'left move'

            info_msg = 'check time: ' + str(check_time_now - self.check_time_pre)

        elif msg.data == 8:
            # TODO : 주행 모드 설정
            driving_state = 'left move'
            mission_time_delay = 0.01

            # TODO : 이미지 및 미션 처리
            check_time_now = time.time()

            # TODO : 미션 판단
            if (check_time_now - self.check_time_pre) > 0.5:
                rospy.loginfo('Order : [seq.8] Goal')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(9)
                driving_state = 'lane following'

            info_msg = 'check time: ' + str(check_time_now - self.check_time_pre)

        elif msg.data == 9:
            # TODO : flag 초기화
            driving_state = 'lane following'
            self.fn_init_mission()

            # TODO : 종료 카운트
            if self.end_count >= 5:
                self.end_count = 0
                rospy.loginfo('Order : [seq.9] mission success !!')
                self.pub_seq_change.publish(100) # 100 : mission end point
                driving_state = 'lane following'
            else:
                self.end_count += 1

            # TODO : 디버깅
            info_msg = 'end count: ' + str(self.end_count)

        self.fn_driving_mode(driving_state)
        #process_time_now = time.time()
        #rospy.loginfo('[seq: ' + str(msg.data) + '] processing time: ' + str(process_time_now - process_time_pre) + '\n>> ' + info_msg)
        if msg.data < 200:
            self.pub_delay_change.publish(mission_time_delay)
            self.pub_timeout_change.publish(mission_timeout)
            rospy.loginfo('[construction][seq: ' + str(msg.data) + ']' + '\n>> ' + info_msg)

    def fn_driving_mode(self, driving_state):
        if driving_state == 'lane following':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_shift_value.publish(0.0) #미션별로 값주기 (pixel)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.lane_mode.value)

        elif driving_state == 'right lane following':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_shift_value.publish(0.0) #미션별로 값주기 (pixel)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.right_lane_mode.value)

        elif driving_state == 'go straight':
            self.pub_linear_value.publish(0.20) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(0.0) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'right move':
            self.pub_linear_value.publish(0.15) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(-2.4) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'left move':
            self.pub_linear_value.publish(0.15) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(2.4) #미션별로 값주기 (rad/s)
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
    rospy.init_node('Construction_Mission_Node')
    node = ConstructionMissionNode()
    node.main()

