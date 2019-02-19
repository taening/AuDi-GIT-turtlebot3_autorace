#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import Image, CompressedImage
from mission_processor import MissionProcessor
from intersection_detector import IntersectionDetector
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import rospy
import cv2
import time
from tf import transformations
import math


class IntersectionMissionNode:
    def __init__(self):
        # TODO : 주차 미션 객체
        self.processor = MissionProcessor()
        self.intersection = IntersectionDetector()
        self.cvBridge = CvBridge()

        # TODO : 미션 처리에 필요한 data
        self.img_ori = None
        self.img_trans = None
        self.img_binary = None
        self.odom_pos_current = [0, 0]
        self.odom_pos_start = [0, 0]
        self.odom_theta_current = 0.0
        self.odom_theta_last = 0.0
        self.odom_theta_start = 0.0
        self.detect_left_lane = False
        self.detect_right_lane = False
        self.max_intensity = 0
        self.finish_crossroad = False
        self.check_time_pre = time.time()
        self.flag_odom_start = False
        self.flag_exit = False
        self.straight_time = 0.0
        self.direction_sign = 'none'
        self.direction_corner = 'none'

        self.end_count = 0

        # TODO : 일반 주행을 위한 미션 노드
        self.sub_ord_rev = rospy.Subscriber('/controller/seq/intersection', UInt8, self.cb_order_receive, queue_size=1)

        self.sub_img_rev = rospy.Subscriber('/controller/image/mission', CompressedImage, self.cb_img_receive, queue_size=1)
        self.sub_lane_trans = rospy.Subscriber('/driving/image/trans', CompressedImage, self.cb_lane_trans, queue_size=1)
        self.sub_lane_binary = rospy.Subscriber('/driving/image/binary', CompressedImage, self.cb_lane_binary, queue_size=1)
        self.sub_odom_info = rospy.Subscriber('/odom', Odometry, self.cb_odom_info, queue_size=1)
        self.sub_left_lane = rospy.Subscriber('/driving/signal/detect_left_lane', UInt8, self.cb_left_lane, queue_size=1)
        self.sub_right_lane = rospy.Subscriber('/driving/signal/detect_right_lane', UInt8, self.cb_right_lane, queue_size=1)
        self.sub_intensity_value = rospy.Subscriber('/driving/value/max_intensity', UInt8, self.cb_max_intensity, queue_size=1)
        self.sub_finish_crossroad = rospy.Subscriber('/driving/signal/finish_crossroad', UInt8, self.cb_finish_crossroad, queue_size=1)
        self.sub_detect_sign = rospy.Subscriber('/sign/signal/detect_sign', UInt8, self.cb_detect_sign, queue_size=1)

        self.pub_driving_mode = rospy.Publisher('/mission/mod/driving', UInt8, queue_size=1)
        self.pub_crossroad_mode = rospy.Publisher('/mission/mod/crossroad', UInt8, queue_size=1)
        self.pub_angular_value = rospy.Publisher('/mission/value/angular_vel', Float32, queue_size=1)
        self.pub_linear_value = rospy.Publisher('/mission/value/linear_vel', Float32, queue_size=1)
        self.pub_shift_value = rospy.Publisher('/mission/value/error_shift', Float32, queue_size=1)

        self.pub_seq_change = rospy.Publisher('/mission/seq/change', UInt8, queue_size=1)
        self.pub_delay_change = rospy.Publisher('/mission/time/delay', Float32, queue_size=1)
        self.pub_timeout_change = rospy.Publisher('/mission/time/timeout', Float32, queue_size=1)

        #self.pub_img_debug = rospy.Publisher('/mission/debug/intersection/compressed', CompressedImage, queue_size=1)
        self.pub_img_debug = rospy.Publisher('/mission/debug/all/compressed', CompressedImage, queue_size=1)

    def fn_init_mission(self):
        self.img_ori = None
        self.img_trans = None
        self.img_binary = None
        self.odom_pos_current = [0, 0]
        self.odom_pos_start = [0, 0]
        self.odom_theta_current = 0.0
        self.odom_theta_last = 0.0
        self.odom_theta_start = 0.0
        self.detect_left_lane = False
        self.detect_right_lane = False
        self.max_intensity = 0
        self.finish_crossroad = False
        self.check_time_pre = time.time()
        self.flag_odom_start = False
        self.flag_exit = False
        self.straight_time = 0.0
        self.direction_sign = 'none'
        self.direction_corner = 'none'

    def cb_img_receive(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.img_ori = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def cb_lane_trans(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.img_trans = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def cb_lane_binary(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.img_binary = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def cb_odom_info(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.odom_theta_current = transformations.euler_from_quaternion(quaternion)[2]

        if (self.odom_theta_current - self.odom_theta_last) < -math.pi:
            self.odom_theta_current = 2. * math.pi + self.odom_theta_current
            self.odom_theta_last = math.pi
        elif (self.odom_theta_current - self.odom_theta_last) > math.pi:
            self.odom_theta_current = -2. * math.pi + self.odom_theta_current
            self.odom_theta_last = -math.pi
        else:
            self.odom_theta_last = self.odom_theta_current

        self.odom_pos_current = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def cb_left_lane(self, msg):
        self.detect_left_lane = bool(msg.data)

    def cb_right_lane(self, msg):
        self.detect_right_lane = bool(msg.data)

    def cb_finish_crossroad(self, msg):
        self.finish_crossroad = bool(msg.data)

    def cb_max_intensity(self, msg):
        self.max_intensity = msg.data

    def cb_detect_sign(self, msg):
        if msg.data == 7:
            self.direction_sign = 'right'
        elif msg.data == 8:
            self.direction_sign = 'left'

    def cb_order_receive(self, msg):
        driving_state = 'none'
        mission_time_delay = 0.1
        mission_timeout = 0.0
        info_msg = ''

        if msg.data == 90:
            # TODO : 이미지 및 미션 처리
            if not self.flag_odom_start:
                self.odom_theta_start = self.odom_theta_current
                self.flag_odom_start = True

            # TODO : 미션 판단
            if True:
                rospy.loginfo('Order : [seq.pre] Intersection Sign detect')
                self.pub_seq_change.publish(95)

            info_msg = ''

        elif msg.data == 99:
            # TODO : 초기화
            rospy.loginfo('Order : [seq.init] init mission')
            self.fn_init_mission()

        elif msg.data == 1:
            # TODO : 주행 모드 설정
            driving_state = 'lane following'
            mission_time_delay = 0.05

            # TODO : 이미지 및 미션 처리
            odom_theta = self.processor.fn_cal_odom_angle(self.odom_theta_current, self.odom_theta_start)

            # TODO : 미션 판단
            if odom_theta > (150 *math.pi/180):
                rospy.loginfo('Order : [seq.1] In the three-way intersection')
                self.pub_seq_change.publish(2)

            info_msg = 'direction_sign: {}'.format(self.direction_sign) + ', odom theta: {0:6f}'.format(odom_theta /math.pi*180)

        elif msg.data == 2:
            # TODO : 주행 모드 설정
            driving_state = 'drive ccw'
            mission_time_delay = 0.05

            # TODO : 이미지 및 미션 처리
            odom_theta = self.processor.fn_cal_odom_angle(self.odom_theta_current, self.odom_theta_start)
            flag_intersection_line, img_intersection_line = self.intersection.fn_find_intersection_line(self.img_trans)

            # TODO : 미션 판단
            if flag_intersection_line:
                rospy.loginfo('Order : [seq.2A] Three-way intersection start')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(4)

            elif odom_theta > (170 *math.pi/180):
                rospy.loginfo('Order : [seq.2B] Go straight')
                self.pub_seq_change.publish(3)

            info_msg = 'direction_sign: {}'.format(self.direction_sign) + ', flag of intersection line: {}'.format(flag_intersection_line) + ', odom theta: {0:6f}'.format(odom_theta /math.pi*180)
            self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_intersection_line, "jpg"))

        elif msg.data == 3:
            # TODO : 주행 모드 설정
            driving_state = 'go straight'

            # TODO : 이미지 및 미션 처리
            flag_intersection_line, img_intersection_line = self.intersection.fn_find_intersection_line(self.img_trans)

            # TODO : 미션 판단
            if flag_intersection_line:
                rospy.loginfo('Order : [seq.3] Three-way intersection start')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(4)

            info_msg = 'direction_sign: {}'.format(self.direction_sign) + ', flag of intersection line: {}'.format(flag_intersection_line)
            self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_intersection_line, "jpg"))

        elif msg.data == 4:
            # TODO : 주행 모드 설정
            driving_state = 'stop'

            # TODO : 이미지 및 미션 처리
            check_time = time.time() - self.check_time_pre
            flag_left, flag_right, img_direction = self.intersection.fn_find_direction_sign(self.img_ori)

            if flag_left:
                self.direction_sign = 'left'
            elif flag_right:
                self.direction_sign = 'right'
            elif check_time >= 2.0:
                self.direction_sign = 'left'

            # TODO : 미션 판단
            if self.direction_sign == 'left':
                rospy.loginfo('Order : [seq.4A] Left crossroad sign')
                self.direction_corner = 'left'
                self.pub_crossroad_mode.publish(self.processor.crossroad_mode_step.l_of_lr_corner.value)
                self.finish_crossroad = False
                self.pub_seq_change.publish(5)

            elif self.direction_sign == 'right':
                rospy.loginfo('Order : [seq.4B] Right crossroad sign')
                self.direction_corner = 'right'
                self.pub_crossroad_mode.publish(self.processor.crossroad_mode_step.r_of_lr_corner.value)
                self.finish_crossroad = False
                self.pub_seq_change.publish(5)

            else:
                self.direction_sign = 'none'####

            # TODO : 디버깅
            info_msg = 'direction_sign: {}'.format(self.direction_sign) + ', direction sign detect: {} {}'.format(flag_left, flag_right) + ', time: {0:.6f}'.format(check_time)
            self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_direction, "jpg"))

        elif msg.data == 5:
            # TODO : 주행 모드 설정
            driving_state = 'lane following'

            # TODO : 이미지 및 미션 처리

            # TODO : 미션 판단
            if self.finish_crossroad:
                rospy.loginfo('Order : [seq.5] Crossroad end')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(6)

            # TODO : 디버깅
            info_msg = 'direction_corner: {}'.format(self.direction_corner) + ', crossroad finish: {}'.format(self.finish_crossroad)

        elif msg.data == 6:
            # TODO : 주행 모드 설정
            driving_state = 'lane following'

            # TODO : 이미지 및 미션 처리
            check_time = time.time() - self.check_time_pre
            odom_theta = self.processor.fn_cal_odom_angle(self.odom_theta_current, self.odom_theta_start)

            # TODO : 미션 판단
            if odom_theta > (170 *math.pi/180):
                rospy.loginfo('Order : [seq.6] Exit three-way intersection')
                self.finish_crossroad = False
                self.pub_seq_change.publish(7)

            # TODO : 디버깅
            info_msg = 'direction_corner: {}'.format(self.direction_corner) + ', time: {0:.6f}'.format(check_time) + ', odom theta: {0:.6f}'.format(odom_theta /math.pi*180)

        elif msg.data == 7:
            # TODO : 주행 모드 설정
            if self.direction_corner == 'left':
                driving_state = 'drive right'
            elif self.direction_corner == 'right':
                driving_state = 'drive left'
            mission_time_delay = 0.05

            # TODO : 이미지 및 미션 처리
            odom_theta = self.processor.fn_cal_odom_angle(self.odom_theta_current, self.odom_theta_start)
            detect_exit_line, (pos_x, _), img_exit_line = self.intersection.fn_find_exit_line(self.img_trans, direction=self.direction_corner)

            # TODO : 미션 판단
            #if odom_theta < (100 *math.pi/180):
            if odom_theta < (110 *math.pi/180) and detect_exit_line:
                rospy.loginfo('Order : [seq.7] Detect exit line')
                if self.direction_corner == 'left':
                    #self.straight_time = float(pos_x - 140) / 120
                    self.pub_crossroad_mode.publish(self.processor.crossroad_mode_step.l_of_sl_corner.value)
                elif self.direction_corner == 'right':
                    #self.straight_time = float(50 - pos_x) / 120
                    self.pub_crossroad_mode.publish(self.processor.crossroad_mode_step.r_of_sr_corner.value)
                driving_state = 'go straight'
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(8)

            # TODO : 디버깅
            info_msg = 'direction_corner: {}'.format(self.direction_corner) + ', odom theta: {0:.6f}'.format(odom_theta /math.pi*180)
            self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_exit_line, "jpg"))

        elif msg.data == 8:
            # TODO : 주행 모드 설정
            if self.direction_corner == 'left':
                #driving_state = 'drive left 2'
                driving_state = 'left lane following'
            elif self.direction_corner == 'right':
                #driving_state = 'drive right 2'
                driving_state = 'right lane following'
            mission_time_delay = 0.02

            # TODO : 이미지 및 미션 처리
            odom_theta = self.processor.fn_cal_odom_angle(self.odom_theta_current, self.odom_theta_start)
            check_time = time.time() - self.check_time_pre

            if check_time < self.straight_time:
                driving_state = 'go straight'

            # TODO : 미션 판단
            #if odom_theta > (150 *math.pi/180):
            if self.finish_crossroad:
                rospy.loginfo('Order : [seq.8] End crossroad driving')
                self.flag_exit = False
                self.check_time_pre = time.time()
                self.end_count = 0
                #self.pub_seq_change.publish(9)
                self.pub_seq_change.publish(10)

            # TODO : 디버깅
            #info_msg = 'direction_corner: {}'.format(self.direction_corner) + ', odom theta: {0:.6f}'.format(odom_theta /math.pi*180) + ', stratight time: {0:.4f} / {1:.4f}'.format(check_time, self.straight_time)
            info_msg = 'direction_corner: {}'.format(self.direction_corner) + ', crossroad finish: {}'.format(self.finish_crossroad)

        elif msg.data == 9:
            # TODO : 주행 모드 설정
            if self.flag_exit:
                if self.direction_corner == 'left':
                    driving_state = 'drive exit for left'
                elif self.direction_corner == 'right':
                    driving_state = 'drive exit for right'
            else:
                if self.direction_corner == 'left':
                    driving_state = 'drive left 2'
                elif self.direction_corner == 'right':
                    driving_state = 'drive right 2'

            # TODO : 이미지 및 미션 처리
            odom_theta = self.processor.fn_cal_odom_angle(self.odom_theta_current, self.odom_theta_start)
            if odom_theta > (160 *math.pi/180):
                self.flag_exit = True

            # TODO : 미션 판단
            if self.detect_left_lane and self.max_intensity > 180:
                rospy.loginfo('Order : [seq.9] End intersection')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(10)

            # TODO : 디버깅
            #info_msg = 'direction_sign: {}'.format(self.direction_sign) + ', crossroad finish: {}'.format(self.finish_crossroad)
            info_msg = 'direction_corner: {}'.format(self.direction_corner) + ', odom theta: {0:.6f}'.format(odom_theta /math.pi*180)

        elif msg.data == 10:
            # TODO : 주행 모드 설정
            driving_state = 'lane following'
            mission_time_delay = 0.03

            # TODO : 이미지 및 미션 처리
            check_time = time.time() - self.check_time_pre

            # TODO : 미션 판단
            if check_time > 10.0:
                rospy.loginfo('Order : [seq.10] Next mission')
                self.end_count = 0
                self.pub_seq_change.publish(11)

            # TODO : 디버깅
            info_msg = 'time: {0:.6f}'.format(check_time)

        elif msg.data == 11:
            # TODO : flag 초기화
            driving_state = 'lane following'
            self.fn_init_mission()

            # TODO : 종료 카운트
            if self.end_count >= 5:
                self.end_count = 0
                rospy.loginfo('Order : [seq.11] mission success !!')
                self.pub_seq_change.publish(100) # 100 : mission end point
            else:
                self.end_count += 1

            # TODO : 디버깅
            info_msg = 'end count: {}'.format(self.end_count)

        self.fn_driving_mode(driving_state)
        if msg.data < 90:
            self.pub_delay_change.publish(mission_time_delay)
            self.pub_timeout_change.publish(mission_timeout)
            rospy.loginfo('[intersection][seq: ' + str(msg.data) + ']' + '\n>> ' + info_msg)

    def fn_driving_mode(self, driving_state):
        if driving_state == 'lane following':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_shift_value.publish(0.0) #미션별로 값주기 (pixel)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.lane_mode.value)

        elif driving_state == 'left lane following':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_shift_value.publish(0.0) #미션별로 값주기 (pixel)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.left_lane_mode.value)

        elif driving_state == 'right lane following':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_shift_value.publish(0.0) #미션별로 값주기 (pixel)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.right_lane_mode.value)

        elif driving_state == 'crossroad lane following':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_shift_value.publish(0.0) #미션별로 값주기 (pixel)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.intensity_lane_mode.value)

        elif driving_state == 'drive left':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(1.5) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'drive left 2':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(1.7) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'drive right':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(-1.5) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'drive right 2':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(-1.7) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'drive exit for left':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(-1.35) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'drive exit for right':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(-1.6) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'drive ccw':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(1.4) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'drive slowly ccw':
            self.pub_linear_value.publish(0.15) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(0.7) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'drive slowly':
            self.pub_linear_value.publish(0.15) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(0.0) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'go straight':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(0.0) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'stop':
            self.pub_linear_value.publish(0.0) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(0.0) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'turn left corner':
            self.pub_linear_value.publish(0.13) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(0.63) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'turn right corner':
            self.pub_linear_value.publish(0.13) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(-0.63) #미션별로 값주기 (rad/s)
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
    rospy.init_node('Intersection_Mission_Node')
    node = IntersectionMissionNode()
    node.main()

