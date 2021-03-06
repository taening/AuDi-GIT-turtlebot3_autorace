#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import Image, CompressedImage
from mission_processor import MissionProcessor
from parking_detector import ParkingDetector
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import rospy
import cv2
import time
from tf import transformations
import math


class ParkingMissionNode:
    def __init__(self):
        # TODO : 주차 미션 객체
        self.processor = MissionProcessor()
        self.parking = ParkingDetector()
        self.cvBridge = CvBridge()

        # TODO : 미션 처리에 필요한 data
        self.img_ori = None
        self.img_trans = None
        self.img_binary = None
        self.scan_dis = [0.0] * 360
        self.odom_pos_current = [0, 0]
        self.odom_pos_start = [0, 0]
        self.odom_theta_current = 0.0
        self.odom_theta_last = 0.0
        self.odom_theta_start = 0.0
        self.finish_crossroad = False
        self.check_time_pre = time.time()
        self.check_left_dot = False
        self.check_right_dot = False
        self.parking_area_direction = 'none'

        self.end_count = 0

        # TODO : 일반 주행을 위한 미션 노드
        self.sub_ord_rev = rospy.Subscriber('/controller/seq/parking', UInt8, self.cb_order_receive, queue_size=1)

        self.sub_img_rev = rospy.Subscriber('/controller/image/mission', CompressedImage, self.cb_img_receive, queue_size=1)
        self.sub_parking_trans = rospy.Subscriber('/driving/image/trans', CompressedImage, self.cb_parking_trans, queue_size=1)
        self.sub_parking_binary = rospy.Subscriber('/driving/image/binary', CompressedImage, self.cb_parking_binary, queue_size=1)
        self.sub_scan_dis = rospy.Subscriber('/scan', LaserScan, self.cb_scan_dis, queue_size=1)
        self.sub_odom_info = rospy.Subscriber('/odom', Odometry, self.cb_odom_info, queue_size=1)
        self.sub_finish_crossroad = rospy.Subscriber('/driving/signal/finish_crossroad', UInt8, self.cb_finish_crossroad, queue_size=1)

        self.pub_driving_mode = rospy.Publisher('/mission/mod/driving', UInt8, queue_size=1)
        self.pub_crossroad_mode = rospy.Publisher('/mission/mod/crossroad', UInt8, queue_size=1)
        self.pub_angular_value = rospy.Publisher('/mission/value/angular_vel', Float32, queue_size=1)
        self.pub_linear_value = rospy.Publisher('/mission/value/linear_vel', Float32, queue_size=1)
        self.pub_shift_value = rospy.Publisher('/mission/value/error_shift', Float32, queue_size=1)

        self.pub_seq_change = rospy.Publisher('/mission/seq/change', UInt8, queue_size=1)
        self.pub_delay_change = rospy.Publisher('/mission/time/delay', Float32, queue_size=1)
        self.pub_timeout_change = rospy.Publisher('/mission/time/timeout', Float32, queue_size=1)

        #self.pub_img_debug = rospy.Publisher('/mission/debug/parking/compressed', CompressedImage, queue_size=1)
        self.pub_img_debug = rospy.Publisher('/mission/debug/all/compressed', CompressedImage, queue_size=1)

    def fn_init_mission(self):
        self.img_ori = None
        self.img_trans = None
        self.img_binary = None
        self.scan_dis = [0.0] * 360
        self.odom_pos_current = [0, 0]
        self.odom_pos_start = [0, 0]
        self.odom_theta_current = 0.0
        self.odom_theta_last = 0.0
        self.odom_theta_start = 0.0
        self.finish_crossroad = False
        self.check_time_pre = time.time()
        self.check_left_dot = False
        self.check_right_dot = False
        self.parking_area_direction = 'none'

    def cb_img_receive(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.img_ori = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def cb_scan_dis(self, msg):
        self.scan_dis = msg.ranges

    def cb_parking_trans(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.img_trans = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def cb_parking_binary(self, msg):
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

    def cb_finish_crossroad(self, msg):
        self.finish_crossroad = bool(msg.data)

    def cb_order_receive(self, msg):
        driving_state = 'none'
        mission_time_delay = 0.1
        mission_timeout = 0.0
        info_msg = ''

        if msg.data == 90:
            # TODO : 이미지 및 미션 처리

            # TODO : 미션 판단
            if True:
                rospy.loginfo('Order : [seq.pre] Parking Sign detect')
                self.finish_crossroad = False
                self.pub_crossroad_mode.publish(self.processor.crossroad_mode_step.l_of_sl_corner.value)
                self.pub_seq_change.publish(92)

            info_msg = ''

        elif msg.data == 99:
            # TODO : 초기화
            rospy.loginfo('Order : [seq.init] init mission')
            self.fn_init_mission()

        elif msg.data == 1:
            # TODO : 주행 모드 설정
            driving_state = 'lane following'

            # TODO : 이미지 및 미션 처리

            # TODO : 미션 판단
            if self.finish_crossroad:
                rospy.loginfo('Order : [seq.1] Entrance parking lot')
                self.check_left_dot = False
                self.check_right_dot = False
                self.pub_seq_change.publish(2)

            # TODO : 디버깅
            info_msg = 'flag of finish crossroad: {}'.format(self.finish_crossroad)

        elif msg.data == 2:
            # TODO : 주행 모드 설정
            driving_state = 'lane following'

            # TODO : 이미지 및 미션 처리
            flag_left_dot, img_left_dot = self.parking.fn_find_dot(self.img_trans, direction='left')
            flag_right_dot, img_right_dot = self.parking.fn_find_dot(self.img_trans, direction='right')
            img_dot = np.hstack((img_left_dot, img_right_dot))

            if flag_left_dot:
                self.check_left_dot = True
            if flag_right_dot:
                self.check_right_dot = True

            # TODO : 미션 판단
            if self.check_left_dot or self.check_right_dot:
                rospy.loginfo('Order : [seq.2] Dot detect')
                self.pub_seq_change.publish(3)

            info_msg = 'flag of lane dot: {0} {1}'.format(self.check_left_dot, self.check_right_dot)
            self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_dot, "jpg"))

        elif msg.data == 3:
            # TODO : 주행 모드 설정
            driving_state = 'lane following'

            # TODO : 이미지 및 미션 처리
            flag_stop, img_stop = self.parking.fn_find_stop(self.img_trans)
            left_scan_count = self.processor.fn_cal_scan_count(self.scan_dis[20:100], 0.4)
            right_scan_count = self.processor.fn_cal_scan_count(self.scan_dis[260:340], 0.4)

            # TODO : 미션 판단
            if flag_stop:
                if left_scan_count < 3:
                    if right_scan_count < 3:
                        rospy.logwarn('Parking : Not detect obstacle')
                    rospy.loginfo('Order : [seq.3A] Left parking area detect')
                    self.odom_theta_start = self.odom_theta_current
                    self.check_time_pre = time.time()
                    self.parking_area_direction = 'left'
                    self.pub_seq_change.publish(4)
                elif right_scan_count < 3:
                    rospy.loginfo('Order : [seq.3B] Right parking area detect')
                    self.odom_theta_start = self.odom_theta_current
                    self.check_time_pre = time.time()
                    self.parking_area_direction = 'right'
                    self.pub_seq_change.publish(4)
                else:
                    rospy.loginfo('Order : [seq.2C] Can\'t detect parking area')
                    #driving_state = 'stop'

            # TODO : 디버깅
            info_msg = 'flag of lane stop: {}'.format(flag_stop) + ', left scan count: {}'.format(left_scan_count) + ', right scan count: {}'.format(right_scan_count)
            self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_stop, "jpg"))

        elif msg.data == 4:
            # TODO : 주행 모드 설정
            mission_time_delay = 0.02
            time_limit = 1.05
            if self.parking_area_direction == 'left':
                driving_state = 'turn left'
                time_limit = 0.6#1.1
            elif self.parking_area_direction == 'right':
                driving_state = 'turn right'
                time_limit = 0.6#1.1

            # TODO : 이미지 및 미션 처리
            check_time = time.time() - self.check_time_pre
            odom_theta = self.processor.fn_cal_odom_angle(self.odom_theta_current, self.odom_theta_start)

            # TODO : 미션 판단
            if check_time > time_limit:
            #if odom_theta > (85 *math.pi/180):
                rospy.loginfo('Order : [seq.4] Parking ready')
                self.odom_pos_start = self.odom_pos_current
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(5)

            # TODO : 디버깅
            info_msg = 'check time: {0:.6f}'.format(check_time) + ', odom theta: {0:.6f}'.format(odom_theta /math.pi*180)

        elif msg.data == 5:
            # TODO : 주행 모드 설정
            mission_time_delay = 0.02
            driving_state = 'go straight'

            # TODO : 이미지 및 미션 처리
            check_time = time.time() - self.check_time_pre
            odom_dis = self.processor.fn_cal_odom_dis(self.odom_pos_start, self.odom_pos_current)

            # TODO : 미션 판단
            if (odom_dis > 0.27) or (check_time > 1.6): # 0.27 1.6
                rospy.loginfo('Order : [seq.5] Insert parking area')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(6)

            # TODO : 디버깅
            info_msg = 'check time: {0:.6f}'.format(check_time) + ', odom dis: {0:.6f}'.format(odom_dis)

        elif msg.data == 6:
            # TODO : 주행 모드 설정
            mission_time_delay = 0.02
            driving_state = 'stop'

            # TODO : 이미지 및 미션 처리
            check_time = time.time() - self.check_time_pre

            # TODO : 미션 판단
            if check_time > 0.02:
                rospy.loginfo('Order : [seq.6] Wait parking')
                self.odom_pos_start = self.odom_pos_current
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(7)

            # TODO : 디버깅
            info_msg = 'check time: {0:.6f}'.format(check_time)

        elif msg.data == 7:
            # TODO : 주행 모드 설정
            mission_time_delay = 0.02
            driving_state = 'go back'

            # TODO : 이미지 및 미션 처리
            check_time = time.time() - self.check_time_pre
            odom_dis = self.processor.fn_cal_odom_dis(self.odom_pos_start, self.odom_pos_current)

            # TODO : 미션 판단
            if (odom_dis > 0.27) or (check_time > 1.6): # 0.25 1.3
                rospy.loginfo('Order : [seq.7] Exit parking area')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(8)

            # TODO : 디버깅
            info_msg = 'check time: {0:.6f}'.format(check_time) + ', odom dis: {0:.6f}'.format(odom_dis)

        elif msg.data == 8:
            # TODO : 주행 모드 설정
            mission_time_delay = 0.02
            time_limit = 1.05
            if self.parking_area_direction == 'left':
                driving_state = 'turn left'
                time_limit = 0.6#0.95
            elif self.parking_area_direction == 'right':
                driving_state = 'turn right'
                time_limit = 0.6#1.0

            # TODO : 이미지 및 미션 처리
            check_time = time.time() - self.check_time_pre
            odom_theta = self.processor.fn_cal_odom_angle(self.odom_theta_current, self.odom_theta_start)

            # TODO : 미션 판단
            if check_time > time_limit:
            #if odom_theta > (175 *math.pi/180):
                rospy.loginfo('Order : [seq.8] return lane following')
                self.check_time_pre = time.time()
                self.pub_seq_change.publish(9)

            # TODO : 디버깅
            info_msg = 'check time: {0:.6f}'.format(check_time) + ', odom theta: {0:.6f}'.format(odom_theta /math.pi*180)

        elif msg.data == 9:
            # TODO : 주행 모드 설정
            mission_time_delay = 0.02
            driving_state = 'lane following'

            # TODO : 이미지 및 미션 처리
            check_time = time.time() - self.check_time_pre
            if check_time < 0.7:
                driving_state = 'go straight'

            # TODO : 미션 판단
            if check_time > 3.0:
                rospy.loginfo('Order : [seq.9] Driving crossroad')
                self.finish_crossroad = False
                self.pub_crossroad_mode.publish(self.processor.crossroad_mode_step.l_of_lr_corner.value)
                self.pub_seq_change.publish(10)

            # TODO : 디버깅
            info_msg = 'check time: {0:.6f}'.format(check_time)

        elif msg.data == 10:
            # TODO : 주행 모드 설정
            driving_state = 'lane following'

            # TODO : 이미지 및 미션 처리

            # TODO : 미션 판단
            if self.finish_crossroad:
                rospy.loginfo('Order : [seq.10] Exit parking lot')
                self.end_count = 0
                self.pub_seq_change.publish(11)

            # TODO : 디버깅
            info_msg = 'flag of finish crossroad: {}'.format(self.finish_crossroad)

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
            rospy.loginfo('[parking][seq: ' + str(msg.data) + ']' + '\n>> ' + info_msg)

    def fn_driving_mode(self, driving_state):
        if driving_state == 'lane following':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_shift_value.publish(0.0) #미션별로 값주기 (pixel)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.lane_mode.value)

        elif driving_state == 'stop':
            self.pub_linear_value.publish(0.0) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(0.0) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'turn right':
            self.pub_linear_value.publish(0.0) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(-2.5) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'turn left':
            self.pub_linear_value.publish(0.0) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(2.5) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'go straight':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_angular_value.publish(0.0) #미션별로 값주기 (rad/s)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.manual_mode.value)

        elif driving_state == 'go back':
            self.pub_linear_value.publish(-0.22) #미션별로 값주기 (m/s)
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
    rospy.init_node('Parking_Mission_Node')
    node = ParkingMissionNode()
    node.main()

