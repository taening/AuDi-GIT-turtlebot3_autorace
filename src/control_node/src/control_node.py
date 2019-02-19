#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import Image, CompressedImage
import enum
import time
import rospy
from cv_bridge import CvBridge


class ControlNode:
    def __init__(self):
        self.traffic_mission_start = False
        self.parking_mission_start = False
        self.crossbar_mission_start = False
        self.tunnel_mission_start = False
        self.intersection_mission_start = False
        self.construction_mission_start = False

        self.traffic_mission_success = False
        self.parking_mission_success = False
        self.crossbar_mission_success = False
        self.tunnel_mission_success = False
        self.intersection_mission_success = False
        self.construction_mission_success = False

        self.mode_step = enum.Enum('step_of_mode', 'normal_mode traffic_mode parking_mode crossbar_mode tunnel_mode intersection_mode construction_mode')
        #self.mode_num = self.mode_step.normal_mode.value
        self.mode_num = self.mode_step.traffic_mode.value
        #self.mode_num = self.mode_step.crossbar_mode.value
        #self.mode_num = self.mode_step.parking_mode.value
        #self.mode_num = self.mode_step.tunnel_mode.value
        #self.mode_num = self.mode_step.intersection_mode.value
        #self.mode_num = self.mode_step.construction_mode.value
        self.sequence_num = 1

        self.driving_time_pre = time.time()
        self.mission_time_pre = time.time()
        self.sign_check_time_pre = time.time()
        self.pre_check_time_pre = time.time()
        self.mission_timeout_pre = time.time()

        self.mission_time_delay = 0.1
        self.mission_timeout = 0.0

        self.img_status = 'compressed' # give 'raw' or 'compressed'

        if self.img_status == 'raw':
            self.cv_bridge = CvBridge()
            self.sub_img_cam = rospy.Subscriber('/image_raw', Image, self.cb_image_receive, queue_size=1)
        elif self.img_status == 'compressed':
            self.sub_img_cam = rospy.Subscriber('/image_raw/compressed', CompressedImage, self.cb_image_receive, queue_size=1)

        self.sub_seq_change = rospy.Subscriber('/mission/seq/change', UInt8, self.cb_sequence_num, queue_size=1)
        self.sub_delay_change = rospy.Subscriber('/mission/time/delay', Float32, self.cb_delay_change, queue_size=1)
        self.sub_timeout_change = rospy.Subscriber('/mission/time/timeout', Float32, self.cb_timeout_change, queue_size=1)

        self.pub_img_driving = rospy.Publisher('/controller/image/driving', CompressedImage, queue_size=1)
        self.pub_img_sign = rospy.Publisher('/controller/image/sign', CompressedImage, queue_size=1)
        self.pub_img_mission = rospy.Publisher('/controller/image/mission', CompressedImage, queue_size=1)

        self.pub_seq_normal = rospy.Publisher('/controller/seq/normal', UInt8, queue_size=1)
        self.pub_seq_traffic = rospy.Publisher('/controller/seq/traffic', UInt8, queue_size=1)
        self.pub_seq_parking = rospy.Publisher('/controller/seq/parking', UInt8, queue_size=1)
        self.pub_seq_crossbar = rospy.Publisher('/controller/seq/crossbar', UInt8, queue_size=1)
        self.pub_seq_tunnel = rospy.Publisher('/controller/seq/tunnel', UInt8, queue_size=1)
        self.pub_seq_intersection = rospy.Publisher('/controller/seq/intersection', UInt8, queue_size=1)
        self.pub_seq_construction = rospy.Publisher('/controller/seq/construction', UInt8, queue_size=1)

        loop_rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.mode_num == self.mode_step.normal_mode.value:
                self.fn_normal_publish()
            else:
                self.fn_mission_publish()
            loop_rate.sleep()

    def fn_normal_publish(self):
        time_now = time.time()
        if self.traffic_mission_start:
            rospy.loginfo('mode change : traffic mode')
            self.mission_timeout_pre = time_now
            self.mode_num = self.mode_step.traffic_mode.value
            self.sequence_num = 1
        elif self.parking_mission_start:
            rospy.loginfo('mode change : parking mode')
            self.mission_timeout_pre = time_now
            self.mode_num = self.mode_step.parking_mode.value
            self.sequence_num = 1
        elif self.crossbar_mission_start:
            rospy.loginfo('mode change : crossbar mode')
            self.mission_timeout_pre = time_now
            self.mode_num = self.mode_step.crossbar_mode.value
            self.sequence_num = 1
        elif self.tunnel_mission_start:
            rospy.loginfo('mode change : tunnel mode')
            self.mission_timeout_pre = time_now
            self.mode_num = self.mode_step.tunnel_mode.value
            self.sequence_num = 1
        elif self.intersection_mission_start:
            rospy.loginfo('mode change : intersection mode')
            self.mission_timeout_pre = time_now
            self.mode_num = self.mode_step.intersection_mode.value
            self.sequence_num = 1
        elif self.construction_mission_start:
            rospy.loginfo('mode change : construction mode')
            self.mission_timeout_pre = time_now
            self.mode_num = self.mode_step.construction_mode.value
            self.sequence_num = 1

        if (time_now - self.mission_time_pre) >= 0.1:
            # rospy.loginfo('[normal] mission sequence publish, time: {0:.4f}'.format(time_now - self.mission_time_pre))
            self.mission_time_pre = time_now
            self.pub_seq_normal.publish(self.sequence_num)

        # TODO: 미션 스타트 지점 퍼블리시
        if (time_now - self.pre_check_time_pre) >= 0.1:
            # rospy.loginfo('    pre check sequence publish, time: {0:.4f}'.format(time_now - self.pre_check_time_pre))
            self.pre_check_time_pre = time_now
            if not self.traffic_mission_success:
                self.pub_seq_traffic.publish(90)
            if not self.parking_mission_success and self.construction_mission_success:
                self.pub_seq_parking.publish(90)
            if not self.crossbar_mission_success and self.parking_mission_success:
                self.pub_seq_crossbar.publish(90)
            if not self.tunnel_mission_success and self.crossbar_mission_success:
                self.pub_seq_tunnel.publish(90)
            if not self.intersection_mission_success and self.traffic_mission_success:
                self.pub_seq_intersection.publish(90)
            if not self.construction_mission_success and self.intersection_mission_success:
                self.pub_seq_construction.publish(90)

    def fn_mission_publish(self):
        time_now = time.time()

        if self.mode_num == self.mode_step.traffic_mode.value:
            if (time_now - self.mission_time_pre) >= self.mission_time_delay:
                #rospy.loginfo('traffic mission sequence publish, time: ' + "{0:.4f}".format(time_now - self.mission_time_pre))
                self.mission_time_pre = time_now
                self.pub_seq_traffic.publish(self.sequence_num)

        elif self.mode_num == self.mode_step.parking_mode.value:
            if (time_now - self.mission_time_pre) >= self.mission_time_delay:
                #rospy.loginfo('parking mission sequence publish, time: ' + "{0:.4f}".format(time_now - self.mission_time_pre))
                self.mission_time_pre = time_now
                self.pub_seq_parking.publish(self.sequence_num)

        elif self.mode_num == self.mode_step.crossbar_mode.value:
            if (time_now - self.mission_time_pre) >= self.mission_time_delay:
                #rospy.loginfo('crossbar mission sequence publish, time: ' + "{0:.4f}".format(time_now - self.mission_time_pre))
                self.mission_time_pre = time_now
                self.pub_seq_crossbar.publish(self.sequence_num)

        elif self.mode_num == self.mode_step.tunnel_mode.value:
            if (time_now - self.mission_time_pre) >= self.mission_time_delay:
                #rospy.loginfo('tunnel mission sequence publish, time: ' + "{0:.4f}".format(time_now - self.mission_time_pre))
                self.mission_time_pre = time_now
                self.pub_seq_tunnel.publish(self.sequence_num)

        elif self.mode_num == self.mode_step.intersection_mode.value:
            if (time_now - self.mission_time_pre) >= self.mission_time_delay:
                #rospy.loginfo('intersection mission sequence publish, time: ' + "{0:.4f}".format(time_now - self.mission_time_pre))
                self.mission_time_pre = time_now
                self.pub_seq_intersection.publish(self.sequence_num)

        elif self.mode_num == self.mode_step.construction_mode.value:
            if (time_now - self.mission_time_pre) >= self.mission_time_delay:
                #rospy.loginfo('construction mission sequence publish, time: ' + "{0:.4f}".format(time_now - self.mission_time_pre))
                self.mission_time_pre = time_now
                self.pub_seq_construction.publish(self.sequence_num)

        else:
            if (time_now - self.mission_time_pre) >= self.mission_time_delay:
                rospy.logerr('[Error] Wrong Mission Mode')

        if self.mission_timeout > 0.1 and (time_now - self.mission_timeout_pre) > self.mission_timeout:
            rospy.logwarn('[warning !!] mode change fail !!')
            self.traffic_pre_check = False
            self.traffic_sign = False
            self.pub_seq_traffic.publish(99)
            self.pub_seq_parking.publish(99)
            self.pub_seq_crossbar.publish(99)
            self.pub_seq_tunnel.publish(99)
            self.pub_seq_intersection.publish(99)
            self.pub_seq_construction.publish(99)
            self.mode_num = self.mode_step.normal_mode.value
            self.sequence_num = 1

    def cb_image_receive(self, msg):
        time_now = time.time()
        if self.img_status == 'raw':
            img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            msg = self.cv_bridge.cv2_to_compressed_imgmsg(img, "jpg")

        # TODO: 표지판 이미지 퍼블리시
        if (time_now - self.sign_check_time_pre) >= 0.01 and self.mode_num == self.mode_step.intersection_mode.value:
            #rospy.loginfo('    sign image publish, time: ' + "{0:.4f}".format(time_now - self.sign_check_time_pre))
            self.sign_check_time_pre = time_now
            self.pub_img_sign.publish(msg)

        # TODO: 드라이빙 이미지 퍼블리시
        if (time_now - self.driving_time_pre) >= 0.1:
            #rospy.loginfo('    driving image publish, time: ' + "{0:.4f}".format(time_now - self.driving_time_pre))
            self.driving_time_pre = time_now
            self.pub_img_driving.publish(msg)

        # TODO: 미션 이미지 퍼블리시
        self.pub_img_mission.publish(msg)

    def cb_sequence_num(self, msg):
        rospy.loginfo('sequence change : ' + str(msg.data))

        if msg.data == 100: # squence end point
            self.traffic_mission_start = False
            self.parking_mission_start = False
            self.crossbar_mission_start = False
            self.tunnel_mission_start = False
            self.intersection_mission_start = False
            self.construction_mission_start = False

            if self.mode_num == self.mode_step.traffic_mode.value:
                self.traffic_mission_success = True
            elif self.mode_num == self.mode_step.parking_mode.value:
                self.parking_mission_success = True
            elif self.mode_num == self.mode_step.crossbar_mode.value:
                self.crossbar_mission_success = True
            elif self.mode_num == self.mode_step.tunnel_mode.value:
                self.tunnel_mission_success = True
            elif self.mode_num == self.mode_step.intersection_mode.value:
                self.intersection_mission_success = True
            elif self.mode_num == self.mode_step.construction_mode.value:
                self.construction_mission_success = True
            self.mode_num = self.mode_step.normal_mode.value
            self.sequence_num = 1

        elif msg.data == 91:
            self.traffic_mission_start = True
        elif msg.data == 92:
            self.parking_mission_start = True
        elif msg.data == 93:
            self.crossbar_mission_start = True
        elif msg.data == 94:
            self.tunnel_mission_start = True
        elif msg.data == 95:
            self.intersection_mission_start = True
        elif msg.data == 96:
            self.construction_mission_start = True

        else:
            self.sequence_num = msg.data
            if self.mode_num == self.mode_step.traffic_mode.value:
                self.pub_seq_traffic.publish(self.sequence_num)
            elif self.mode_num == self.mode_step.parking_mode.value:
                self.pub_seq_parking.publish(self.sequence_num)
            elif self.mode_num == self.mode_step.crossbar_mode.value:
                self.pub_seq_crossbar.publish(self.sequence_num)
            elif self.mode_num == self.mode_step.tunnel_mode.value:
                self.pub_seq_tunnel.publish(self.sequence_num)
            elif self.mode_num == self.mode_step.intersection_mode.value:
                self.pub_seq_intersection.publish(self.sequence_num)
            elif self.mode_num == self.mode_step.construction_mode.value:
                self.pub_seq_construction.publish(self.sequence_num)

        self.mission_timeout_pre = time.time()

    def cb_delay_change(self, msg):
        self.mission_time_delay = msg.data

    def cb_timeout_change(self, msg):
        self.mission_timeout = msg.data

    @staticmethod
    def main():
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Control_Node')
    node = ControlNode()
    node.main()


