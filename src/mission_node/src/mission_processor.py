#!/usr/bin/env python
# -*- coding: utf-8 -*-
import enum
import math
import numpy as np
import cv2


class MissionProcessor:
    def __init__(self):
        self.driving_mode_step = enum.Enum('step_of_driving_mode', 'manual_mode lane_mode right_lane_mode left_lane_mode intensity_lane_mode deeplearning_lane_mode tunnel_mode spectate_mode')
        self.crossroad_mode_step = enum.Enum('step_of_crossroad_mode', 'just_follow_lane s_of_sl_corner s_of_sr_corner l_of_sl_corner l_of_lr_corner r_of_sr_corner r_of_lr_corner')
        self.detect_sign_step = enum.Enum('step_of_detect_sign', 'traffic intersection construction parking crossbar tunnel left right')

    def fn_cal_odom_dis(self, pos1, pos2):
        return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

    def fn_cal_odom_angle(self, theta1, theta2):
        #theta = theta2 - theta1
        theta = abs(theta2 - theta1)
        if theta > 2*math.pi:
            theta -= 2*math.pi
        elif theta > math.pi:
            theta = 2*math.pi - theta
        #elif theta < -math.pi:
        #    theta = -2*math.pi + theta
        return theta

    def fn_cal_scan_count(self, list_scan_dis, max_dis):
        scan_count = 0
        for scan_dis in list_scan_dis:
            if 0 < scan_dis < max_dis:
                scan_count += 1
        return scan_count

    def fn_cal_ultrasonic_dis(self, list_ultrasonic_dis):
        list_dis = []
        for ultrasonic_dis in list_ultrasonic_dis:
            if ultrasonic_dis != 0.0:
                list_dis.append(ultrasonic_dis)
        if len(list_dis) > 0:
            mean_dis = sum(list_dis) / len(list_dis)
        else:
            mean_dis = 0.0
        return mean_dis
