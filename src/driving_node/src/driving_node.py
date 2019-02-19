#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import Image, CompressedImage
from velocity_controller import VelocityController
from lane_processor import LaneProcessor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import rospy
import cv2
import enum
import time
import copy
from tf import transformations
import math


class DrivingNode:
    def __init__(self):
        self.controller = VelocityController()
        self.lane = LaneProcessor()
        self.cvBridge = CvBridge()

        self.driving_mode_step = enum.Enum('step_of_driving_mode', 'manual_mode lane_mode right_lane_mode left_lane_mode intensity_lane_mode deeplearning_lane_mode tunnel_mode spectate_mode')
        self.driving_mode = self.driving_mode_step.manual_mode.value

        self.crossroad_mode_step = enum.Enum('step_of_crossroad_mode', 'just_follow_lane s_of_sl_corner s_of_sr_corner l_of_sl_corner l_of_lr_corner r_of_sr_corner r_of_lr_corner')
        self.crossroad_mode = self.crossroad_mode_step.just_follow_lane.value

        self.flag_crossroad_driving = False
        self.flag_crossroad_corner = False

        self.flag_init_tunnel = False

        self.scan_dis = [0.0] * 360
        self.odom_pos_current = [0.0, 0.0]
        self.odom_pos_start = [0.0, 0.0]
        self.odom_theta_current = 0.0
        self.odom_theta_last = 0.0
        self.odom_theta_start = 0.0

        self.mode = 0
        self.turn = 0
        self.time_pre = 0
        self.time_turn = 0
        self.turn_time = 0
        self.num = 0

        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.error_shift = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.val_threshold = 220

        self.sub_driving_mode = rospy.Subscriber('/mission/mod/driving', UInt8, self.cb_driving_mode, queue_size=1)
        self.sub_crossroad_mode = rospy.Subscriber('/mission/mod/crossroad', UInt8, self.cb_crossroad_mode, queue_size=1)

        self.sub_linear_value = rospy.Subscriber('/mission/value/linear_vel', Float32, self.cb_linear_vel, queue_size=1)
        self.sub_angular_value = rospy.Subscriber('/mission/value/angular_vel', Float32, self.cb_angular_vel, queue_size=1)
        self.sub_shift_value = rospy.Subscriber('/mission/value/error_shift', Float32, self.cb_error_shift, queue_size=1)
        self.sub_threshold_value = rospy.Subscriber('/mission/value/lane_threshold', UInt8, self.cb_lane_threshold, queue_size=1)

        self.sub_driving_image = rospy.Subscriber('/controller/image/driving', CompressedImage, self.cb_driving_image, queue_size=1)
        self.sub_driving_odom = rospy.Subscriber('/odom', Odometry, self.cb_driving_odom, queue_size=1)
        self.sub_driving_scan = rospy.Subscriber('/scan', LaserScan, self.cb_driving_scan)

        self.pub_img_trans = rospy.Publisher('/driving/image/trans', CompressedImage, queue_size=1)
        self.pub_img_binary = rospy.Publisher('/driving/image/binary', CompressedImage, queue_size=1)
        self.pub_left_lane = rospy.Publisher('/driving/signal/detect_left_lane', UInt8, queue_size=1)
        self.pub_right_lane = rospy.Publisher('/driving/signal/detect_right_lane', UInt8, queue_size=1)
        self.pub_finish_crossroad = rospy.Publisher('/driving/signal/finish_crossroad', UInt8, queue_size=1)
        self.pub_intensity_value = rospy.Publisher('/driving/value/max_intensity', UInt8, queue_size=1)

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.pub_img_debug = rospy.Publisher('/driving/debug/lane/compressed', CompressedImage, queue_size=1)

        self.flag_save = False  #TODO : True로 주면 주행하는 이미지를 모두 저장함. 경로지정이 잘못됬거나 저장할 이유가 없으면 False로 둘 것.

        if self.flag_save:
            #self.save_img_path = '/home/nvidia/Auto-Mobile-Robot-Final/data/'
            self.save_img_path = '/media/nvidia/9016-4EF8/data/'
            f = open((self.save_img_path + 'count.txt'), 'r')
            lines = f.readlines()
            try:
                self.save_img_count = int(lines[0])
            except:
                rospy.loginfo("count init")
                self.save_img_count = 0
            rospy.loginfo("count : " + str(self.save_img_count))
            f.close()

        rospy.on_shutdown(self.fn_stop)

        loop_rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.fn_driving_timer()
            loop_rate.sleep()

    def fn_cal_distance(self, pos1, pos2):
        return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

    def fn_cal_angle(self, theta1, theta2):
        theta = abs(theta2 - theta1)
        if theta > 2*math.pi:
            theta -= 2*math.pi
        elif theta > math.pi:
            theta = 2*math.pi - theta
        return theta

    def fn_detect_crossroad(self, flag_left_lane, flag_right_lane, max_intensity):
        flag_crossroad_start = False

        if self.crossroad_mode == self.crossroad_mode_step.just_follow_lane:
            pass
        elif self.crossroad_mode == self.crossroad_mode_step.s_of_sl_corner.value or self.crossroad_mode == self.crossroad_mode_step.l_of_sl_corner.value:
            if not flag_left_lane:
                flag_crossroad_start = True
        elif self.crossroad_mode == self.crossroad_mode_step.s_of_sr_corner.value or self.crossroad_mode == self.crossroad_mode_step.r_of_sr_corner.value:
            if not flag_right_lane:
                flag_crossroad_start = True
        elif self.crossroad_mode == self.crossroad_mode_step.l_of_lr_corner.value or self.crossroad_mode == self.crossroad_mode_step.r_of_lr_corner.value:
            if (not flag_left_lane) or (not flag_right_lane) or (max_intensity < 200):
                flag_crossroad_start = True

        return flag_crossroad_start

    def fn_crossroad_driving(self, img_trans, img_threshold, max_intensity, direction='none', corner='none'):
        flag_complete = False

        if self.flag_crossroad_corner:
            if direction == 'straight':
                if corner == 'sl':
                    err, _, _ = self.lane.fn_lane_process(img_trans, img_threshold, left_lane=False)
                elif corner == 'sr':
                    err, _, _ = self.lane.fn_lane_process(img_trans, img_threshold, right_lane=False)
                else:
                    rospy.logerr('direction order error')
                    err = 0.0
                twist_msg = self.controller.fn_pid_control(self.linear_vel, err, self.kp, 0.0, self.kd)
                err_lane, [flag_left_lane, flag_right_lane], img_debug = self.lane.fn_lane_process(img_trans, img_threshold)
                odom_dis = self.fn_cal_distance(self.odom_pos_current, self.odom_pos_start)
                if odom_dis > 0.25 and flag_left_lane and flag_right_lane:
                    flag_complete = True
                    twist_msg = self.controller.fn_pid_control(self.linear_vel, err_lane, self.kp, 0.0, self.kd)

            elif direction == 'left':
                #twist_msg = self.controller.fn_vel_control(0.22, 2.5)
                twist_msg = self.controller.fn_vel_control(0.2, 2.2)
                self.lane.fn_init_parameter()
                err_lane, [flag_left_lane, flag_right_lane], img_debug = self.lane.fn_lane_process(img_trans, img_threshold)
                odom_angle = self.fn_cal_angle(self.odom_theta_current, self.odom_theta_start)
                if (odom_angle > (60 *math.pi/180) and flag_right_lane) or (odom_angle > (75 *math.pi/180) and flag_left_lane):
                    flag_complete = True
                    twist_msg = self.controller.fn_pid_control(self.linear_vel, err_lane, self.kp, 0.0, self.kd)

            elif direction == 'right':
                #twist_msg = self.controller.fn_vel_control(0.22, -2.5)
                twist_msg = self.controller.fn_vel_control(0.2, -2.2)
                self.lane.fn_init_parameter()
                err_lane, [flag_left_lane, flag_right_lane], img_debug = self.lane.fn_lane_process(img_trans, img_threshold)
                odom_angle = self.fn_cal_angle(self.odom_theta_current, self.odom_theta_start)
                print(odom_angle)
                if odom_angle > (60 *math.pi/180) and flag_left_lane or (odom_angle > (75 *math.pi/180) and flag_right_lane):
                    flag_complete = True
                    twist_msg = self.controller.fn_pid_control(self.linear_vel, err_lane, self.kp, 0.0, self.kd)

            else:
                rospy.logerr('you select direction (straight, left, right)')
                twist_msg = self.controller.fn_vel_control(0.0, 0.0)
                self.lane.fn_init_parameter()
                _, _, img_debug = self.lane.fn_lane_process(img_trans, img_threshold)

        else:
            if corner == 'sr':
                err, _, _ = self.lane.fn_lane_process(img_trans, img_threshold, right_lane=False)
                twist_msg = self.controller.fn_pid_control(self.linear_vel, err, self.kp, 0.0, self.kd)
                flag_crossroad_corner, img_debug = self.lane.fn_detect_crossroad_corner(img_threshold, corner_direction='right')
                if flag_crossroad_corner and max_intensity > 180:
                    self.odom_pos_start = self.odom_pos_current
                    self.odom_theta_start = self.odom_theta_current
                    self.flag_crossroad_corner = True
                    twist_msg, flag_complete, img_debug = self.fn_crossroad_driving(img_trans, img_threshold, max_intensity, direction, corner)

            elif corner == 'sl':
                err, _, _ = self.lane.fn_lane_process(img_trans, img_threshold, left_lane=False)
                twist_msg = self.controller.fn_pid_control(self.linear_vel, err, self.kp, 0.0, self.kd)
                flag_crossroad_corner, img_debug = self.lane.fn_detect_crossroad_corner(img_threshold, corner_direction='left')
                if flag_crossroad_corner and max_intensity > 180:
                    self.odom_pos_start = self.odom_pos_current
                    self.odom_theta_start = self.odom_theta_current
                    self.flag_crossroad_corner = True
                    twist_msg, flag_complete, img_debug = self.fn_crossroad_driving(img_trans, img_threshold, max_intensity, direction, corner)

            elif corner == 'lr':
                twist_msg = self.controller.fn_vel_control(self.linear_vel, 0.0)
                flag_crossroad_corner, img_debug = self.lane.fn_detect_crossroad_line(img_threshold)
                if flag_crossroad_corner and max_intensity > 180:
                    self.odom_pos_start = self.odom_pos_current
                    self.odom_theta_start = self.odom_theta_current
                    self.flag_crossroad_corner = True
                    twist_msg, flag_complete, img_debug = self.fn_crossroad_driving(img_trans, img_threshold, max_intensity, direction, corner)

            else:
                rospy.logerr('you select corner (sr, sl, rl)')
                twist_msg = self.controller.fn_vel_control(0.0, 0.0)
                self.lane.fn_init_parameter()
                _, _, img_debug = self.lane.fn_lane_process(img_trans, img_threshold)

        return twist_msg, flag_complete, img_debug

    def fn_manual_driving(self):
        twist_msg = self.controller.fn_vel_control(self.linear_vel, self.angular_vel)
        return twist_msg

    def fn_auto_driving(self, img_trans, img_threshold, lane_mode='none'):
        if lane_mode == 'right':
            err, flag_lane, img_debug = self.lane.fn_lane_process(img_trans, img_threshold, left_lane=False)
        elif lane_mode == 'left':
            err, flag_lane, img_debug = self.lane.fn_lane_process(img_trans, img_threshold, right_lane=False)
        else:
            err, flag_lane, img_debug = self.lane.fn_lane_process(img_trans, img_threshold)

        err += self.error_shift

        twist_msg = self.controller.fn_pid_control(self.linear_vel, err, self.kp, 0.0, self.kd)
        return twist_msg, flag_lane, img_debug

    def fn_intensity_driving(self, img_trans, img_threshold, max_intensity, intensity_limit = 180):
        err, flag_lane, img_debug = self.lane.fn_lane_process(img_trans, img_threshold)

        if max_intensity > intensity_limit:
            err += self.error_shift
            twist_msg = self.controller.fn_pid_control(self.linear_vel, err, self.kp, 0.0, self.kd)
        else:
            twist_msg = self.controller.fn_vel_control(self.linear_vel, 0.0)

        return twist_msg, flag_lane, img_debug

    def fn_tunnel_driving(self, list_scan_dis, odom_theta_current):
        if not self.flag_init_tunnel:
            self.controller.fn_tunnel_init(odom_theta_current)
            self.flag_init_tunnel = True
        twist_msg = self.controller.fn_tunnel_control(list_scan_dis, odom_theta_current)
        #twist_msg = self.controller.fn_kh_control(list_scan_dis)
        #twist_msg = self.controller.fn_frog_control(list_scan_dis)

        return twist_msg

    def fn_lane_detect(self, img_trans, img_threshold):
        self.lane.fn_init_parameter()
        _, [flag_left_lane, flag_right_lane], img_debug = self.lane.fn_lane_process(img_trans, img_threshold)

        return flag_left_lane, flag_right_lane, img_debug

    def fn_stop(self):
        rospy.loginfo("[Process End] Wait other process...")
        self.sub_driving_image.unregister()
        self.sub_driving_odom.unregister()
        self.sub_driving_scan.unregister()
        rospy.sleep(0.5)
        rospy.loginfo("[Process End] Shut down")
        twist_msg = self.controller.fn_vel_control(0.0, 0.0)
        self.pub_cmd_vel.publish(twist_msg)

    def fn_save_image(self, img_ori, twist_msg):
        # img_ori = cv2.resize(img_ori, (80, 60), interpolation=cv2.INTER_NEAREST)
        cv2.imwrite((self.save_img_path + 'img/{}.jpg').format(self.save_img_count), img_ori)
        f = open((self.save_img_path + 'vel/lane.txt'), 'a')
        f.write(str(self.save_img_count) + ', ' + str(twist_msg.linear.x) + ', ' + str(twist_msg.angular.z) + '\n')
        f.close()
        self.save_img_count += 1
        f = open((self.save_img_path + 'count.txt'), 'w')
        f.write(str(self.save_img_count))
        f.close()

    def cb_driving_image(self, msg):
        process_time_pre = time.time()

        np_arr = np.fromstring(msg.data, np.uint8)
        img_ori = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img_roi = self.lane.fn_lane_roi(img_ori)
        img_trans = self.lane.fn_trans_perspective(img_roi)
        self.pub_img_trans.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_trans, "jpg"))
        img_threshold, max_intensity, hist = self.lane.fn_lane_threshold(img_trans, self.val_threshold)
        self.pub_img_binary.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv2.merge((img_threshold, img_threshold, img_threshold)), "jpg"))
        self.pub_intensity_value.publish(max_intensity)

        if self.flag_crossroad_driving:
            if self.crossroad_mode == self.crossroad_mode_step.s_of_sl_corner.value:
                twist_msg, flag_crossroad_complete, img_debug = self.fn_crossroad_driving(img_trans, img_threshold, max_intensity, direction='straight', corner='sl')
                self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_debug, "jpg"))
                self.pub_cmd_vel.publish(twist_msg)

            elif self.crossroad_mode == self.crossroad_mode_step.s_of_sr_corner.value:
                twist_msg, flag_crossroad_complete, img_debug = self.fn_crossroad_driving(img_trans, img_threshold, max_intensity, direction='straight', corner='sr')
                self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_debug, "jpg"))
                self.pub_cmd_vel.publish(twist_msg)

            elif self.crossroad_mode == self.crossroad_mode_step.l_of_sl_corner.value:
                twist_msg, flag_crossroad_complete, img_debug = self.fn_crossroad_driving(img_trans, img_threshold, max_intensity, direction='left', corner='sl')
                self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_debug, "jpg"))
                self.pub_cmd_vel.publish(twist_msg)

            elif self.crossroad_mode == self.crossroad_mode_step.l_of_lr_corner.value:
                twist_msg, flag_crossroad_complete, img_debug = self.fn_crossroad_driving(img_trans, img_threshold, max_intensity, direction='left', corner='lr')
                self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_debug, "jpg"))
                self.pub_cmd_vel.publish(twist_msg)

            elif self.crossroad_mode == self.crossroad_mode_step.r_of_sr_corner.value:
                twist_msg, flag_crossroad_complete, img_debug = self.fn_crossroad_driving(img_trans, img_threshold, max_intensity, direction='right', corner='sr')
                self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_debug, "jpg"))
                self.pub_cmd_vel.publish(twist_msg)

            elif self.crossroad_mode == self.crossroad_mode_step.r_of_lr_corner.value:
                twist_msg, flag_crossroad_complete, img_debug = self.fn_crossroad_driving(img_trans, img_threshold, max_intensity, direction='right', corner='lr')
                self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(img_debug, "jpg"))
                self.pub_cmd_vel.publish(twist_msg)

            else:
                flag_crossroad_complete = False
                rospy.logerr('잘못된 갈림길 주행 모드')

            if flag_crossroad_complete:
                self.pub_finish_crossroad.publish(1)
                self.crossroad_mode = self.crossroad_mode_step.just_follow_lane
                self.flag_crossroad_driving = False

        else:
            if self.driving_mode == self.driving_mode_step.manual_mode.value:
                detect_left_lane, detect_right_lane, img_debug = self.fn_lane_detect(img_trans, img_threshold)
                self.pub_left_lane.publish(int(detect_left_lane))
                self.pub_right_lane.publish(int(detect_right_lane))
                self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(np.hstack((img_debug, hist)), "jpg"))

            elif self.driving_mode == self.driving_mode_step.lane_mode.value:
                twist_msg, flag_lane, img_debug = self.fn_auto_driving(img_trans, img_threshold)
                flag_crossroad_start = self.fn_detect_crossroad(flag_lane[0], flag_lane[1], max_intensity)
                if flag_crossroad_start:
                    self.flag_crossroad_corner = False
                    self.flag_crossroad_driving = True
                    twist_msg = self.controller.fn_vel_control(self.linear_vel, 0.0)

                self.pub_cmd_vel.publish(twist_msg)
                self.pub_left_lane.publish(int(flag_lane[0]))
                self.pub_right_lane.publish(int(flag_lane[1]))
                self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(np.hstack((img_debug, hist)), "jpg"))

                #TODO:이미지 저장
                if self.flag_save:
                    self.fn_save_image(img_ori, twist_msg)

            elif self.driving_mode == self.driving_mode_step.right_lane_mode.value:
                twist_msg, flag_lane, img_debug = self.fn_auto_driving(img_trans, img_threshold, lane_mode='right')
                flag_crossroad_start = self.fn_detect_crossroad(flag_lane[0], flag_lane[1], max_intensity)
                if flag_crossroad_start:
                    self.flag_crossroad_corner = False
                    self.flag_crossroad_driving = True
                    twist_msg = self.controller.fn_vel_control(self.linear_vel, 0.0)

                self.pub_cmd_vel.publish(twist_msg)
                self.pub_left_lane.publish(int(flag_lane[0]))
                self.pub_right_lane.publish(int(flag_lane[1]))
                self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(np.hstack((img_debug, hist)), "jpg"))


                #TODO:이미지 저장
                if self.flag_save:
                    self.fn_save_image(img_ori, twist_msg)

            elif self.driving_mode == self.driving_mode_step.left_lane_mode.value:
                twist_msg, flag_lane, img_debug = self.fn_auto_driving(img_trans, img_threshold, lane_mode='left')
                flag_crossroad_start = self.fn_detect_crossroad(flag_lane[0], flag_lane[1], max_intensity)
                if flag_crossroad_start:
                    self.flag_crossroad_corner = False
                    self.flag_crossroad_driving = True
                    twist_msg = self.controller.fn_vel_control(self.linear_vel, 0.0)

                self.pub_cmd_vel.publish(twist_msg)
                self.pub_left_lane.publish(int(flag_lane[0]))
                self.pub_right_lane.publish(int(flag_lane[1]))
                self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(np.hstack((img_debug, hist)), "jpg"))

                #TODO:이미지 저장
                if self.flag_save:
                    self.fn_save_image(img_ori, twist_msg)

            elif self.driving_mode == self.driving_mode_step.intensity_lane_mode.value:
                twist_msg, flag_lane, img_debug = self.fn_intensity_driving(img_trans, img_threshold, max_intensity, 180)
                flag_crossroad_start = self.fn_detect_crossroad(flag_lane[0], flag_lane[1], max_intensity)
                if flag_crossroad_start:
                    self.flag_crossroad_corner = False
                    self.flag_crossroad_driving = True
                    twist_msg = self.controller.fn_vel_control(self.linear_vel, 0.0)

                self.pub_cmd_vel.publish(twist_msg)
                self.pub_left_lane.publish(int(flag_lane[0]))
                self.pub_right_lane.publish(int(flag_lane[1]))
                self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(np.hstack((img_debug, hist)), "jpg"))

            elif self.driving_mode == self.driving_mode_step.deeplearning_lane_mode.value:
                pass

            elif self.driving_mode == self.driving_mode_step.tunnel_mode.value:
                detect_left_lane, detect_right_lane, img_debug = self.fn_lane_detect(img_trans, img_threshold)
                self.pub_left_lane.publish(int(detect_left_lane))
                self.pub_right_lane.publish(int(detect_right_lane))
                self.pub_img_debug.publish(self.cvBridge.cv2_to_compressed_imgmsg(np.hstack((img_debug, hist)), "jpg"))

            elif self.driving_mode == self.driving_mode_step.spectate_mode.value:
                pass

            else:
                rospy.logerr('잘못된 주행 모드')

        rospy.loginfo('process time : {0:.4f}'.format(time.time() - process_time_pre))

    def fn_driving_timer(self):
        if self.driving_mode == self.driving_mode_step.manual_mode.value:
            twist_msg = self.fn_manual_driving()
            self.pub_cmd_vel.publish(twist_msg)

    def cb_driving_odom(self, msg):
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

    def cb_driving_scan(self, msg):
        self.scan_dis = list(msg.ranges)

        if self.driving_mode == self.driving_mode_step.tunnel_mode.value:
            twist_msg = self.fn_tunnel_driving(self.scan_dis, self.odom_theta_current)
            self.pub_cmd_vel.publish(twist_msg)
        else:
            self.flag_init_tunnel = False

    def cb_driving_mode(self, msg):
        self.driving_mode = msg.data

    def cb_crossroad_mode(self, msg):
        self.crossroad_mode = msg.data

    def cb_linear_vel(self, msg):
        self.linear_vel = msg.data

        if self.linear_vel > 0.205:
            #self.kp = 0.0256
            #self.kd = 0.0020
            self.kp = 0.0192
            self.kd = 0.0020
            #self.kp = 0.0200
            #self.kd = 0.0000
        elif self.linear_vel > 0.19:
            self.kp = 0.0156
            self.kd = 0.0016
            #self.kp = 0.0180
            #self.kd = 0.0000
        elif self.linear_vel > 0.16:
            self.kp = 0.0144
            self.kd = 0.0016
        elif self.linear_vel > 0.06:
            self.kp = 0.0072
            self.kd = 0.0012
        else:
            self.kp = 0.0024
            self.kd = 0.0004

    def cb_angular_vel(self, msg):
        self.angular_vel = msg.data

    def cb_error_shift(self, msg):
        self.error_shift = msg.data

    def cb_lane_threshold(self, msg):
        self.val_threshold = msg.data


    @staticmethod
    def main():
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('Driving_Node')
    node = DrivingNode()
    node.main()


