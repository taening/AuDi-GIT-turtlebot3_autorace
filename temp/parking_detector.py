#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import itertools
import math
import cv2


class ParkingDetector:
    def __init__(self):
        self.error_linear_limit = 10.0

    def _fn_dis_dot_and_line(self, const, pos):
        (a, b, c) = const
        (x0, y0) = pos
        distance = abs(x0*a + y0*b + c) / math.sqrt(a*a + b*b)
        return distance

    def _fn_cal_line(self, pos1, pos2):
        [x1, y1] = pos1
        [x2, y2] = pos2

        if x2-x1 == 0:
            a = 1000
        else:
            a = float(y2-y1) / (x2-x1)
        b = -1
        c = y1 - a*x1

        return (a, b, c)

    def fn_find_dot(self, img_trans, direction='left'):
        dotlane_check = False

        # ROI 영역에 맞게 자른 이미지
        pers_height, pers_width = img_trans.shape[:2]  # shape is w384 x h240
        if direction == 'left':
            img_gray = cv2.cvtColor(img_trans[:, :int(pers_width *5/ 8)].copy(), cv2.COLOR_RGB2GRAY)
        else:
            img_gray = cv2.cvtColor(img_trans[:, int(pers_width *3/ 8):].copy(), cv2.COLOR_RGB2GRAY)
        _, img_dot = cv2.threshold(img_gray, 200, 255, 0)
        img_debug = cv2.merge((img_dot, img_dot, img_dot))

        # Dot Lane 변환 및 좌표정보
        _, list_contour_dotlane, _ = cv2.findContours(img_dot, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Dot Lane 이미지 생성
        list_all_dotlane_pos = []

        for contour_dotlane in list_contour_dotlane:
            cv2.drawContours(img_debug, [contour_dotlane], 0, (0, 0, 255), 2)
            x_dot, y_dot, w_dot, h_dot = cv2.boundingRect(contour_dotlane)

            #if (10 < w_dot < 30) and (25 < h_dot < 55) and (1 < (float(h_dot) / w_dot) < 4):
            if (10 < w_dot < 30) and (20 < h_dot < 80) and (2 < (float(h_dot) / w_dot) < 6):
                cv2.drawContours(img_debug, [contour_dotlane], 0, (0, 255, 0), 2)
                moment = cv2.moments(contour_dotlane)
                area = moment['m00']
                if area > 10:
                    cx = int(moment['m10'] / moment['m00'])
                    cy = int(moment['m01'] / moment['m00'])
                    #print(cx, cy, w_dot, h_dot, area)
                    list_all_dotlane_pos.append([cx, cy, w_dot, h_dot, area])

        num_dot = 3
        if len(list_all_dotlane_pos) >= num_dot:
            list_combination_dotlane_pos = list(itertools.combinations(list_all_dotlane_pos, num_dot))

            for list_dotlane_pos in list_combination_dotlane_pos:
                flag_linear = True

                sort_idx = [x for x in range(len(list_dotlane_pos))]
                for ii in range(len(sort_idx)):
                    for jj in range(ii + 1, len(sort_idx)):
                        if list_dotlane_pos[sort_idx[ii]][1] > list_dotlane_pos[sort_idx[jj]][1]:
                            temp = sort_idx[jj]
                            sort_idx[jj] = sort_idx[ii]
                            sort_idx[ii] = temp

                val_line_constant = self._fn_cal_line(list_dotlane_pos[sort_idx[0]][:2], list_dotlane_pos[sort_idx[num_dot-1]][:2])
                for idx_linear in range(num_dot):
                    error_linear = self._fn_dis_dot_and_line(val_line_constant, list_dotlane_pos[sort_idx[idx_linear]][:2])
                    if error_linear > self.error_linear_limit:
                        flag_linear = False

                #error_width = [0] * num_dot
                #for idx_w in range(num_dot):
                #    error_width[idx_w] = list_dotlane_pos[sort_idx[idx_w]][2]
                #error_width_max = max(error_width)
                #error_width_min = min(error_width)
                #if error_width_max - error_width_min > self.error_width_limit:
                #    flag_width = False

                #error_height = [0] * num_dot
                #for idx_h in range(num_dot):
                #    error_height[idx_h] = list_dotlane_pos[sort_idx[idx_h]][3]
                #error_height_max = max(error_height)
                #error_height_min = min(error_height)
                #if error_height_max - error_height_min > self.error_height_limit:
                #    flag_height = False

                #error_area = [0] * num_dot
                #for idx_a in range(num_dot):
                #    error_area[idx_a] = list_dotlane_pos[sort_idx[idx_a]][4]
                #error_area_max = max(error_area)
                #error_area_min = min(error_area)
                #if error_area_max - error_area_min > self.error_area_limit:
                #    flag_area = False

                if flag_linear:
                    cv2.circle(img_debug, tuple(list_dotlane_pos[sort_idx[0]][:2]), 4, (255, 0, 0), 2)
                    for ii in range(1, num_dot-1):
                        cv2.circle(img_debug, tuple(list_dotlane_pos[sort_idx[ii]][:2]), 4, (120, 0, 0), 2)
                    cv2.circle(img_debug, tuple(list_dotlane_pos[sort_idx[num_dot-1]][:2]), 4, (255, 0, 0), 2)

                    #print (list_dotlane_pos[sort_idx[0]][2:5], list_dotlane_pos[sort_idx[1]][2:5], list_dotlane_pos[sort_idx[2]][2:5])

                    gradient = val_line_constant[0]
                    shift_x = int((gradient / math.sqrt(1 + gradient**2)) * self.error_linear_limit)
                    shift_y = int((1 / math.sqrt(1 + gradient**2)) * self.error_linear_limit)
                    cv2.line(img_debug, tuple(list_dotlane_pos[sort_idx[0]][:2]), tuple(list_dotlane_pos[sort_idx[num_dot-1]][:2]), (0, 120, 120), 2)
                    cv2.line(img_debug, (list_dotlane_pos[sort_idx[0]][0] + shift_x, list_dotlane_pos[sort_idx[0]][1] + shift_y),
                             (list_dotlane_pos[sort_idx[num_dot-1]][0] + shift_x, list_dotlane_pos[sort_idx[num_dot-1]][1] + shift_y), (0, 255, 255), 2)
                    cv2.line(img_debug, (list_dotlane_pos[sort_idx[0]][0] - shift_x, list_dotlane_pos[sort_idx[0]][1] - shift_y),
                             (list_dotlane_pos[sort_idx[num_dot-1]][0] - shift_x, list_dotlane_pos[sort_idx[num_dot-1]][1] - shift_y), (0, 255, 255), 2)
                    dotlane_check = True
                    break

        return dotlane_check, img_debug

    def fn_find_stop(self, img_trans):
        # ROI 영역에 맞게 자른 이미지
        pers_height, pers_width = img_trans.shape[:2]  # shape is w384 x h240
        img_gray = cv2.cvtColor(img_trans[int(pers_height * 3/ 8):, :].copy(), cv2.COLOR_RGB2GRAY)
        _, img_stop = cv2.threshold(img_gray, 200, 255, 0)
        img_debug = cv2.merge((img_stop, img_stop, img_stop))

        # Stop 관련
        _, contours_stop, _ = cv2.findContours(img_stop, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        stop_check = False
        # Stop 이미지 생성
        for cnt in contours_stop:  # 점선 인식
            cv2.drawContours(img_debug, [cnt], 0, (0, 0, 255), 2)
            x_stop, y_stop, w_stop, h_stop = cv2.boundingRect(cnt)
            #print(w_stop)
            if 355 < w_stop:  # 주차공간 판단
                cv2.drawContours(img_debug, [cnt], 0, (0, 255, 0), 2)
                #print(w_stop)
                stop_check = True

        return stop_check, img_debug
