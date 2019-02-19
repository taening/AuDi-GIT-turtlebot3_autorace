#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import cv2


class TrafficDetector:
    def __init__(self):
        #self.lower_red = np.array([170,140,130],np.uint8) # 대회
        #self.upper_red = np.array([185,255,255],np.uint8) # 대회

        self.lower_red = np.array([155,120,81],np.uint8)
        self.upper_red = np.array([185,255,255],np.uint8)

        self.lower_yellow = np.array([30,10,80],np.uint8)
        self.upper_yellow = np.array([50,10,80],np.uint8)

        #self.lower_green = np.array([75,110,110], np.uint8)  #대회
        #self.upper_green = np.array([90,255,255], np.uint8)   #대회

        #self.lower_green = np.array([75,140,130], np.uint8)  #대회 pre
        #self.upper_green = np.array([90,255,255], np.uint8)   #대회 pre

        self.lower_green = np.array([82,120,50], np.uint8)
        self.upper_green = np.array([95,255,255], np.uint8)

        self._kernel3 = np.ones((3, 3), np.uint8)  # 컨투어 이용할 번수
        self._kernel7 = np.ones((7, 7), np.uint8)  # 컨투어 이용할 번수

        self.area_ratio_limit = 0.2
        self.min_dis_limit = 50

        self.list_red = []
        self.list_yellow = []
        self.list_green = []

        self.list_light = []

    def fn_init_red(self):
        self.list_red = []

    def fn_init_yellow(self):
        self.list_yellow = []

    def fn_init_green(self):
        self.list_green = []

    def fn_init_light(self):
        self.list_light = []

    def _fn_cal_distance(self, pts1, pts2):
        return math.sqrt((pts2[0]-pts1[0])**2+(pts2[1]-pts1[1])**2)

    def _fn_clip_roi(self, img_ori):
        val_height , val_width = img_ori.shape[0],img_ori.shape[1]  # shape is 480 x 640
        img_roi = img_ori[:val_height*4/9, val_width*2/3:].copy()  # 관심영역 설정
        return img_roi

    def _fn_mask_hsv(self, img_roi, lower_hsv, upper_hsv):
        img_hsv = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, lower_hsv, upper_hsv)
        return img_mask

    def _fn_tracking_find_hough(self, img_mask, min_dis_limit):
        img_blur = cv2.GaussianBlur(img_mask, (5, 5), 0)
        img_canny = cv2.Canny(img_blur, 200, 50)
        keypts = cv2.HoughCircles(img_canny, cv2.HOUGH_GRADIENT, 1, 10, param1=10, param2=10, minRadius=2, maxRadius=30)  # 빨간원검출 파라미터 수정 필요

        list_all_obj = []
        if keypts is not None:
            keypt = keypts[0]
            for [cx, cy, radius] in keypt:
                area = int(radius**2 * math.pi)
                x0 = int(cx - radius)
                y0 = int(cy - radius)
                w = int(radius * 2)
                h = int(radius * 2)
                flag_ok = True
                for cx_pre, cy_pre, _, _, _, _, _ in list_all_obj:
                    if self._fn_cal_distance((cx, cy), (cx_pre, cy_pre)) < min_dis_limit:
                        flag_ok = False
                if flag_ok:
                    list_all_obj.append([cx, cy, area, x0, y0, w, h])

        return list_all_obj

    def _fn_tracking_find_contour(self, img_mask, min_dis_limit, (length_min, length_max), aspect_ratio_limit, area_lower_limit):
        _, list_contour, _ = cv2.findContours(img_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        list_all_obj = []

        for contour in list_contour:
            x0, y0, w, h = cv2.boundingRect(contour)

            if (length_min < w < length_max) and (length_min < h < length_max) and (1/aspect_ratio_limit < (float(w) / h) < aspect_ratio_limit):
                moment = cv2.moments(contour)
                area = moment['m00']
                area_rectacgle = w * h
                area_ratio = float(area) / area_rectacgle
                if area_ratio > area_lower_limit:
                    cx = int(moment['m10'] / moment['m00'])
                    cy = int(moment['m01'] / moment['m00'])
                    flag_ok = True
                    for cx_pre, cy_pre, _, _, _, _, _ in list_all_obj:
                        if self._fn_cal_distance((cx, cy), (cx_pre, cy_pre)) < min_dis_limit:
                            flag_ok = False
                    if flag_ok:
                        list_all_obj.append([cx, cy, area, x0, y0, w, h])

        return list_all_obj

    def _fn_tracking_traffic(self, img_roi, img_mask, traffic_obj, min_dis_limit = 50, miss_limit = 5, (length_min, length_max) = (15, 40), aspect_ratio_limit = 1.5, area_lower_limit = 0.4):
        img_morphology = cv2.morphologyEx(img_mask, cv2.MORPH_OPEN, self._kernel3, iterations=1)
        img_roi = cv2.bitwise_and(img_roi, img_roi, mask=img_morphology)
        img_debug = img_roi

        list_all_obj = self._fn_tracking_find_contour(img_mask, min_dis_limit, (length_min, length_max), aspect_ratio_limit, area_lower_limit)
        #list_all_obj = self._fn_tracking_find_hough(img_mask, min_dis_limit)

        if list_all_obj is not None:
            flag_continue_obj = [False] * len(traffic_obj)
            flag_continue_obj_new = [False] * len(list_all_obj)

            arr_dis = [[10000 for _ in range(len(traffic_obj))] for _ in range(len(list_all_obj))]
            list_min_dis = [10000 for _ in range(len(list_all_obj))]

            for ii_new in range(len(list_all_obj)):
                cx_new, cy_new, area_new, x0_new, y0_new, w_new, h_new = list_all_obj[ii_new]
                for ii in range(len(traffic_obj)):
                    _, _, cx, cy, area, x0, y0, w, h = traffic_obj[ii]
                    if True:
                        dis = self._fn_cal_distance((cx_new, cy_new), (cx, cy))
                        (arr_dis[ii_new])[ii] = dis

            while True:
                for ii_new in range(len(list_all_obj)):
                    if arr_dis[ii_new] == []:
                        min_dis = 10000
                    else:
                        min_dis = min(arr_dis[ii_new])
                    list_min_dis[ii_new] = min_dis
                if list_min_dis == []:
                    final_min_dis = 10000
                else:
                    final_min_dis = min(list_min_dis)

                if final_min_dis > min_dis_limit:
                    break

                idx_obj_new = list_min_dis.index(final_min_dis)
                idx_obj = arr_dis[idx_obj_new].index(list_min_dis[idx_obj_new])

                arr_dis[idx_obj_new] = [10000] * len(traffic_obj)
                for ii_new in range(len(list_all_obj)):
                    arr_dis[ii_new][idx_obj] = 10000

                flag_continue_obj[idx_obj] = True
                flag_continue_obj_new[idx_obj_new] = True

                traffic_obj[idx_obj][0] += 1
                traffic_obj[idx_obj][1] = 0
                traffic_obj[idx_obj][2:] = list_all_obj[idx_obj_new]

            for ii in range(len(traffic_obj)):
                if not flag_continue_obj[ii]:
                    traffic_obj[ii][1] += 1

            ii = 0
            while ii < len(traffic_obj):
                if traffic_obj[ii][1] >= miss_limit:
                    traffic_obj.pop(ii)
                else:
                    ii += 1

            list_new_obj = []
            for ii_new in range(len(list_all_obj)):
                if not flag_continue_obj_new[ii_new]:
                    list_new_obj.append([1, 0] + list_all_obj[ii_new])

            traffic_obj += list_new_obj

        return traffic_obj, img_debug

    def fn_traffic_count_fixed_light(self, img_ori):
        red_light_count = 0
        yellow_light_count = 0
        green_light_count = 0

        img_roi = self._fn_clip_roi(img_ori)
        img_gray = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
        _, img_light = cv2.threshold(img_gray, 200, 255, cv2.THRESH_BINARY)
        self.list_light, img_debug = self._fn_tracking_traffic(img_roi, img_light, self.list_light, 10, 5)

        red_cx_min = 80
        red_cx_max = 135
        red_cy_min = 63
        red_cy_max = 95
        cv2.rectangle(img_debug, (red_cx_min, red_cy_min), (red_cx_max, red_cy_max), (127, 127, 255), 1)

        yellow_cx_min = 78
        yellow_cx_max = 133
        yellow_cy_min = 95
        yellow_cy_max = 128
        cv2.rectangle(img_debug, (yellow_cx_min, yellow_cy_min), (yellow_cx_max, yellow_cy_max), (127, 255, 255), 1)

        green_cx_min = 75
        green_cx_max = 130
        green_cy_min = 128
        green_cy_max = 160
        cv2.rectangle(img_debug, (green_cx_min, green_cy_min), (green_cx_max, green_cy_max), (127, 255, 127), 1)

        for count, miss, cx, cy, area, x0, y0, w, h in self.list_light:
            cv2.rectangle(img_debug, (x0, y0), (x0 + w, y0 + h), (0, 0, 127), 1)
            cv2.putText(img_debug, str(count), (x0, y0+9), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, (255, 0, 0))

            if (red_cx_min < cx < red_cx_max) and (red_cy_min < cy < red_cy_max):
                cv2.rectangle(img_debug, (x0, y0), (x0 + w, y0 + h), (0, 0, 255), 1)
                if count > red_light_count:
                    red_light_count = count

            if (yellow_cx_min < cx < yellow_cx_max) and (yellow_cy_min < cy < yellow_cy_max):
                cv2.rectangle(img_debug, (x0, y0), (x0 + w, y0 + h), (0, 255, 255), 1)
                if count > yellow_light_count:
                    yellow_light_count = count

            if (green_cx_min < cx < green_cx_max) and (green_cy_min < cy < green_cy_max):
                cv2.rectangle(img_debug, (x0, y0), (x0 + w, y0 + h), (0, 255, 0), 1)
                if count > green_light_count:
                    green_light_count = count

        return (red_light_count, yellow_light_count, green_light_count), img_debug

    def fn_traffic_count_green(self, img_ori):
        green_light_count = 0

        img_roi = self._fn_clip_roi(img_ori)
        img_green = self._fn_mask_hsv(img_roi, self.lower_green, self.upper_green)
        self.list_green, img_debug = self._fn_tracking_traffic(img_roi, img_green, self.list_green, 50, 5, (20,80), 2, 0.4)

        for count, miss, cx, cy, area, x0, y0, w, h in self.list_green:
            cv2.rectangle(img_debug, (x0, y0), (x0 + w, y0 + h), (0, 255, 0), 1)
            cv2.putText(img_debug, str(count), (x0, y0+9), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, (255, 0, 0))
            if count > green_light_count:
                green_light_count = count

        return green_light_count, img_debug

    def fn_traffic_count_yellow(self, img_ori):
        yellow_light_count = 0

        img_roi = self._fn_clip_roi(img_ori)
        img_yellow = self._fn_mask_hsv(img_roi, self.lower_yellow, self.upper_yellow)
        self.list_yellow, img_debug = self._fn_tracking_traffic(img_roi, img_yellow, self.list_yellow, 50, 5, (20,80), 2, 0.4)

        for count, miss, cx, cy, area, x0, y0, w, h in self.list_yellow:
            cv2.rectangle(img_debug, (x0, y0), (x0 + w, y0 + h), (0, 255, 0), 1)
            cv2.putText(img_debug, str(count), (x0, y0+9), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, (255, 0, 0))
            if count > yellow_light_count:
                yellow_light_count = count

        return yellow_light_count, img_debug

    def fn_traffic_count_red(self, img_ori):
        red_light_count = 0

        img_roi = self._fn_clip_roi(img_ori)
        img_red = self._fn_mask_hsv(img_roi, self.lower_red, self.upper_red)
        self.list_red, img_debug = self._fn_tracking_traffic(img_roi, img_red, self.list_red, 50, 5, (20,80), 2, 0.4)

        for count, miss, cx, cy, area, x0, y0, w, h in self.list_red:
            cv2.rectangle(img_debug, (x0, y0), (x0 + w, y0 + h), (0, 255, 0), 1)
            cv2.putText(img_debug, str(count), (x0, y0+9), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, (255, 0, 0))
            if count > red_light_count:
                red_light_count = count

        return red_light_count, img_debug

    def fn_traffic_count_green_in_red(self, img_ori, red_count_limit):
        green_light_count = 0

        img_roi = self._fn_clip_roi(img_ori)

        img_green = self._fn_mask_hsv(img_roi, self.lower_green, self.upper_green)
        img_red = self._fn_mask_hsv(img_roi, self.lower_red, self.upper_red)

        self.list_green, img_debug_g = self._fn_tracking_traffic(img_roi, img_green, self.list_green, 50, 5, (20,80), 2, 0.4)
        self.list_red, img_debug_r = self._fn_tracking_traffic(img_roi, img_red, self.list_red, 5, 10000)

        img_debug = cv2.bitwise_or(img_debug_g, img_debug_r)

        list_green = []

        for count, miss, cx, cy, area, x0, y0, w, h in self.list_green:
            cv2.rectangle(img_debug, (x0, y0), (x0 + w, y0 + h), (0, 255, 255), 1)
            cv2.putText(img_debug, str(count), (x0, y0+9), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, (255, 255, 0))
            list_green.append([cx, cy, count])

        for count, miss, cx, cy, area, x0, y0, w, h in self.list_red:
            cv2.rectangle(img_debug, (x0, y0), (x0 + w, y0 + h), (0, 255, 0), 1)
            cv2.putText(img_debug, str(count), (x0, y0+9), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, (255, 255, 0))
            if count >= red_count_limit:
                cx_min = int(cx - 30)
                cx_max = int(cx + 30)
                cy_min = cy + 20
                cy_max = img_ori.shape[0]
                cv2.rectangle(img_debug, (cx_min, cy_min), (cx_max, cy_max), (255, 0, 0), 1)
                for cx_green, cy_green, count_green in list_green:
                    if (cx_min < cx_green < cx_max) and (cy_min < cy_green < cy_max):
                        if count_green > green_light_count:
                            green_light_count = count_green

        return green_light_count, img_debug
