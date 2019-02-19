#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import Image, CompressedImage
from mission_processor import MissionProcessor
from sensor_msgs.msg import LaserScan
import numpy as np
import rospy
import cv2


class NormalMissionNode:
    def __init__(self):
        # TODO : 미션 객체
        self.processor = MissionProcessor()

        # TODO : 미션 처리에 필요한 data
        self.img_ori = None
        #self.scan_dis = [0.0] * 360

        # TODO : 일반 주행을 위한 미션 노드
        self.sub_ord_rev = rospy.Subscriber('/controller/seq/normal', UInt8, self.cb_order_receive, queue_size=1)

        self.sub_img_rev = rospy.Subscriber('/controller/image/mission', CompressedImage, self.cb_img_receive, queue_size=1)
        #self.sub_scan_dis = rospy.Subscriber('/scan', LaserScan, self.cb_scan_dis, queue_size=1)

        self.pub_driving_mode = rospy.Publisher('/mission/mod/driving', UInt8, queue_size=1)
        self.pub_angular_value = rospy.Publisher('/mission/value/angular_vel', Float32, queue_size=1)
        self.pub_linear_value = rospy.Publisher('/mission/value/linear_vel', Float32, queue_size=1)
        self.pub_shift_value = rospy.Publisher('/mission/value/error_shift', Float32, queue_size=1)

    def cb_img_receive(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.img_ori = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    #def cb_scan_dis(self, msg):
    #    self.scan_dis = msg.ranges

    def cb_order_receive(self, msg):
        #process_time_pre = time.time()
        driving_state = 'none'

        if msg.data == 1:
            # TODO : 주행 모드 설정
            driving_state = 'lane following'

            # TODO : 이미지 및 미션 처리
            #scan_count = self.processor.fn_cal_scan_count(self.scan_dis[-15:]+self.scan_dis[:15], 0.25)

            # TODO : 미션 판단
            #if scan_count > 5:
            #    rospy.loginfo('Order : [seq.pre] Detect obstacle')
            #    driving_state = 'stop'
        else:
            rospy.logerr('잘못된 접근')

        self.fn_driving_mode(driving_state)
        #process_time_now = time.time()
        #rospy.loginfo('[seq: ' + str(msg.data) + '] processing time: ' + str(process_time_now - process_time_pre) + '\n>> ' + info_msg)
        rospy.loginfo('[normal] lane following..')

    def fn_driving_mode(self, driving_state):
        if driving_state == 'lane following':
            self.pub_linear_value.publish(0.22) #미션별로 값주기 (m/s)
            self.pub_shift_value.publish(0.0) #미션별로 값주기 (pixel)
            self.pub_driving_mode.publish(self.processor.driving_mode_step.lane_mode.value)

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
    rospy.init_node('Normal_Mission_Node')
    node = NormalMissionNode()
    node.main()


