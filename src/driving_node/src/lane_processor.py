#!/usr/bin/env python
# -*-coding:utf-8-*-
from cv_bridge import CvBridge
import numpy as np
import cv2
import rospy

class LaneProcessor:
    def __init__(self):
        self.bridge = CvBridge()

        self._list_pos_ori = rospy.get_param('/perspective/original', [[156, 0], [-37, 240], [509, 0], [660, 240]])
        self._list_pos_trans = rospy.get_param('/perspective/trans', [[73, 0], [73, 240], [359, 0], [359, 240]])
        self._val_trans_width, self._val_trans_height = rospy.get_param('/perspective/size', [432, 240])
        self._center_x, self._center_y = rospy.get_param('/perspective/center', [215, 190])

        self.cx_left_pre = int(self._val_trans_width / 8)
        self.cx_right_pre = int(self._val_trans_width * 7 / 8)
        self.shift_pre = int((self.cx_right_pre - self.cx_left_pre) / 2)
        self.gap_pre = self.cx_right_pre - self.cx_left_pre

        self.list_right_pts_x, self.list_right_pts_y = [], []
        self.list_left_pts_x, self.list_left_pts_y = [], []

        self.error_pre = 0.0

    def fn_init_parameter(self): # 외부 호출
        self.cx_left_pre = int(self._val_trans_width / 8)
        self.cx_right_pre = int(self._val_trans_width * 7 / 8)
        self.shift_pre = int((self.cx_right_pre - self.cx_left_pre) / 2)
        self.gap_pre = self.cx_right_pre - self.cx_left_pre
        self.error_pre = 0.0

    def _fn_limit_parameter(self):
        if self.shift_pre < self._val_trans_width*11/32:
            self.shift_pre = int(self._val_trans_width*11/32)
        elif self.shift_pre > self._val_trans_width*13/32:
            self.shift_pre = int(self._val_trans_width*13/32)

        if self.cx_left_pre < 0:
            self.cx_left_pre = 0
        elif self.cx_left_pre > (self._val_trans_width * 1 / 2):
            self.cx_left_pre = (self._val_trans_width * 1 / 2)

        if self.cx_right_pre < (self._val_trans_width * 1 / 2):
            self.cx_right_pre = (self._val_trans_width * 1 / 2)
        elif self.cx_right_pre > self._val_trans_width:
            self.cx_right_pre = self._val_trans_width

        if self.gap_pre < self._val_trans_width*11/16:
            self.gap_pre = int(self._val_trans_width*11/16)
        elif self.gap_pre > self._val_trans_width*13/16:
            self.gap_pre = int(self._val_trans_width*13/16)

        if (self.cx_right_pre - self.cx_left_pre) < (self._val_trans_width / 4) :
            gap_now = (self._val_trans_width / 4) - (self.cx_right_pre - self.cx_left_pre)
            self.cx_left_pre -= int(gap_now / 2)
            self.cx_right_pre += int(gap_now / 2)

    def _fn_init_list_pts(self):
        self.list_right_pts_x = []
        self.list_right_pts_y = []
        self.list_left_pts_x = []
        self.list_left_pts_y = []

    def fn_lane_roi(self, img_ori): # 외부 호출
        val_h, val_w = img_ori.shape[:2]
        img_lane = img_ori[int(val_h / 2):, :]
        return img_lane

    def fn_trans_perspective(self, img_ori): # 외부 호출
        img_trans = self.trans_perspective(img_ori, self._list_pos_ori, self._list_pos_trans, self._val_trans_width, self._val_trans_height)
        return img_trans

    def fn_find_max_intensity(self, img, hist_percent): # 외부 호출
        val_hist_size = 256

        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if hist_percent == 0.0:
            _, val_stretch_high, _, _ = cv2.minMaxLoc(img_gray)
        else:
            hist_ori = cv2.calcHist([img_gray], [0], None, [val_hist_size], [0, val_hist_size])
            accumulator = np.cumsum(hist_ori)
            num_of_pixel = accumulator[val_hist_size - 1]
            num_of_clip = num_of_pixel * (hist_percent / 100.)
            val_stretch_high = val_hist_size-1

            for val_stretch_high in range(val_hist_size-1, -1, -1):
                if accumulator[val_stretch_high] <= (num_of_pixel - num_of_clip):
                    break

        return val_stretch_high

    def fn_detect_crossroad_corner(self, img_binary, corner_direction='none'): # 외부 호출
        pers_height, pers_width = img_binary.shape[:2]  # shape is w384 x h240
        img_binary = cv2.morphologyEx(img_binary, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        img_binary = cv2.morphologyEx(img_binary, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        if corner_direction == 'left':
            #img_crossroad = img_binary[int(pers_height * 3/8):int(pers_height * 5/8), :int(pers_width * 1/2)].copy()
            img_crossroad = img_binary[int(pers_height * 0/8):int(pers_height * 7/16), :int(pers_width * 1/2)].copy()
        elif corner_direction == 'right':
            #img_crossroad = img_binary[int(pers_height * 3/8):int(pers_height * 5/8), int(pers_width * 1/2):].copy()
            img_crossroad = img_binary[int(pers_height * 5/16):int(pers_height * 6/8), int(pers_width * 1/2):].copy()
        else:
            raise ValueError('must select reft or right')
        img_debug = cv2.merge((img_crossroad, img_crossroad, img_crossroad))

        _, contours_crossroad, _ = cv2.findContours(img_crossroad, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        crossroad_check = False
        for cnt in contours_crossroad:
            cv2.drawContours(img_debug, [cnt], 0, (0, 0, 255), 2)
            img_mask = np.zeros_like(img_crossroad, dtype=np.uint8)
            cv2.drawContours(img_mask, [cnt], 0, 255, -1)
            cnt_crossroad = np.count_nonzero(img_mask)
            val_x, val_y, val_width, val_height = cv2.boundingRect(cnt)
            cv2.putText(img_debug, 'w: {}, h: {}'.format(val_width, val_height), (cnt[0][0][0]+10, cnt[0][0][1]+10), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (0, 255, 255))
            if cnt_crossroad > 200 and val_width > 40:
                #rospy.logwarn(val_width)
                #rospy.logwarn(cnt_crossroad)
                cv2.drawContours(img_debug, [cnt], 0, (0, 255, 0), 2)
                crossroad_check = True

        return crossroad_check, img_debug

    def fn_detect_crossroad_line(self, img_binary): # 외부 호출
        pers_height, pers_width = img_binary.shape[:2]  # shape is w384 x h240
        img_binary = cv2.morphologyEx(img_binary, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        img_binary = cv2.morphologyEx(img_binary, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        img_crossroad = img_binary[int(pers_height * 3/ 8):int(pers_height * 7/ 8), :].copy()
        img_debug = cv2.merge((img_crossroad, img_crossroad, img_crossroad))

        _, contours_line, _ = cv2.findContours(img_crossroad, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        crossroad_check = False
        for cnt in contours_line:
            cv2.drawContours(img_debug, [cnt], 0, (0, 0, 255), 2)
            val_x, val_y, val_width, val_height = cv2.boundingRect(cnt)
            cv2.putText(img_debug, 'w: {}, h: {}'.format(val_width, val_height), (cnt[0][0][0]+10, cnt[0][0][1]+10), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (0, 255, 255))
            if 250 < val_width:
                cv2.drawContours(img_debug, [cnt], 0, (0, 255, 0), 2)
                crossroad_check = True

        return crossroad_check, img_debug

    @staticmethod
    def hsv_filter(image): # 외부 호출
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # HSV_White color  [0,0,200] [255,255,255]
        # HSV_White color  [70,10,130] [180,110,255]
        lower_white = np.array([0, 0, 200], np.uint8)
        upper_white = np.array([255, 255, 255], np.uint8)
        mask_w = cv2.inRange(hsv, lower_white, upper_white)

        # HSV_Yellow color [20,100,100] [30,255,255]
        lower_yellow = np.array([15, 50, 100], np.uint8)
        upper_yellow = np.array([35, 255, 255], np.uint8)
        mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)

        mask_line = cv2.add(mask_w, mask_y)
        return mask_line

    @staticmethod
    def trans_perspective(image, pos_ori, pos_trans, width, height): # 외부 호출
        arr_pos_ori = np.float32(pos_ori)
        arr_pos_trans = np.float32(pos_trans)
        mat_perspective = cv2.getPerspectiveTransform(arr_pos_ori, arr_pos_trans)
        img_trans = cv2.warpPerspective(image, mat_perspective, (width, height))
        return img_trans

    def _fn_hist_curve(self, img):
        bins = np.arange(256).reshape(256, 1)
        h = np.zeros((self._val_trans_height, 256, 3), dtype=np.uint8)
        if len(img.shape) == 2:
            color = [(255,255,255)]
        elif img.shape[2] == 3:
            color = [ (255,0,0),(0,255,0),(0,0,255) ]
        for ch, col in enumerate(color):
            hist_item = cv2.calcHist([img], [ch], None, [256], [0,256])
            cv2.normalize(hist_item, hist_item, 0, 255, cv2.NORM_MINMAX)
            hist = np.int32(np.around(hist_item))
            pts = np.int32(np.column_stack((bins, hist)))
            cv2.polylines(h, [pts], False, col)
        y = np.flipud(h)
        return y

    def fn_lane_threshold(self, img_trans, val_threshold = 220): # 외부 호출
        #TODO : hsv filter 방식
        #mask_line = self.hsv_filter(img_trans)
        #line = cv2.bitwise_and(img_trans, img_trans, mask=mask_line)

        #img_gray = cv2.cvtColor(line, cv2.COLOR_BGR2GRAY)
        #hist = np.array(self._fn_hist_curve(img_gray))
        #_, img_threshold = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY)

        #TODO : parameter threshold 방식
        #val_threshold = 180
        #img_gray = cv2.cvtColor(img_trans, cv2.COLOR_BGR2GRAY)
        #hist = np.array(self._fn_hist_curve(img_gray))
        #_, img_threshold = cv2.threshold(img_gray, val_threshold, 255, cv2.THRESH_BINARY)
        #cv2.line(hist, (val_threshold, 0), (val_threshold, self._val_trans_height), (0, 0, 255), 3)

        #TODO : otsu threshold 방식
        #img_gray = cv2.cvtColor(img_trans, cv2.COLOR_BGR2GRAY)
        #hist = np.array(self._fn_hist_curve(img_gray))
        #val_threshold, img_threshold = cv2.threshold(img_gray, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        #print('otsu threshold value : ', val_threshold)

        #TODO : otsu threshold 방식2
        #img_gray = cv2.cvtColor(img_trans, cv2.COLOR_BGR2GRAY)
        #hist = np.array(self._fn_hist_curve(img_gray))
        #val_threshold, _ = cv2.threshold(img_gray, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        #val_threshold_new = val_threshold + 30
        #_, img_threshold = cv2.threshold(img_gray, val_threshold_new, 255, cv2.THRESH_BINARY)
        #print('otsu threshold value : ', val_threshold_new)
        #cv2.line(hist, (int(val_threshold), 0), (int(val_threshold), self._val_trans_height), (255, 0, 0), 3)
        #cv2.line(hist, (int(val_threshold_new), 0), (int(val_threshold_new), self._val_trans_height), (0, 0, 255), 3)

        #TODO : end-in stretch + parameter threshold 방식
        val_hist_percent = 1.0 # 3.0
        val_hist_size = 256

            #TODO : gray color
        #val_threshold = 220
        img_1ch = cv2.cvtColor(img_trans, cv2.COLOR_BGR2GRAY)

            #TODO : hsv split
        #val_threshold = 210
        #img_hsv = cv2.cvtColor(img_trans, cv2.COLOR_BGR2HSV)
        #_, _, img_1ch = cv2.split(img_hsv)

            #TODO : R+G color 사용불가
        #val_threshold = 200
        #_, img_g, img_r = cv2.split(img_trans)
        #img_1ch = np.mean((img_g, img_r), axis=0, dtype=np.uint8)

            #TODO : R+G color2
        #val_threshold = 210
        #_, img_g, img_r = cv2.split(img_trans)
        #img_zero = np.zeros_like(img_r, dtype=np.uint8)
        #img_1ch = cv2.cvtColor(cv2.merge((img_zero, img_g, img_r)), cv2.COLOR_BGR2GRAY)

        if val_hist_percent == 0.0:
            val_stretch_low, val_stretch_high, _, _ = cv2.minMaxLoc(img_1ch)
        else:
            hist_ori = cv2.calcHist([img_1ch], [0], None, [val_hist_size], [0, val_hist_size])
            accumulator = np.cumsum(hist_ori)
            num_of_pixel = accumulator[val_hist_size - 1]
            num_of_clip = num_of_pixel * (val_hist_percent / 100.)

            for val_stretch_low in range(val_hist_size):
                if accumulator[val_stretch_low] >= accumulator[0] + num_of_clip:
                    break

            for val_stretch_high in range(val_hist_size-1, -1, -1):
                if accumulator[val_stretch_high] <= (num_of_pixel - num_of_clip):
                    break

        try:
            input_range = val_stretch_high - val_stretch_low
            alpha = float(val_hist_size - 1) / input_range
            beta = -val_stretch_low * alpha

            img_stretch = cv2.convertScaleAbs(img_1ch, -1, alpha, beta)
            _, img_threshold_low = cv2.threshold(img_1ch, val_stretch_low, 255, cv2.THRESH_BINARY)
            _, img_threshold_high = cv2.threshold(img_1ch, val_stretch_high, 255, cv2.THRESH_BINARY)
            img_stretch = cv2.bitwise_or(img_stretch, img_threshold_high, mask=img_threshold_low)
        except:
            img_stretch = np.zeros_like(img_1ch, dtype=np.uint8)

        hist_1 = np.array(self._fn_hist_curve(img_1ch))
        hist_2 = np.array(self._fn_hist_curve(img_stretch))
        cv2.line(hist_1, (int(val_stretch_high), 0), (int(val_stretch_high), self._val_trans_height), (255, 0, 0), 1)
        cv2.line(hist_1, (int(val_stretch_low), 0), (int(val_stretch_low), self._val_trans_height), (0, 255, 255), 1)
        cv2.putText(hist_1, 'high: {}'.format(val_stretch_high), (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(hist_1, 'low: {}'.format(val_stretch_low), (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.line(hist_2, (int(val_threshold), 0), (int(val_threshold), self._val_trans_height), (0, 0, 255), 3)
        rospy.logwarn(val_threshold)
        img_gray_debug = cv2.merge((img_1ch, img_1ch, img_1ch))
        img_stretch_debug = cv2.merge((img_stretch, img_stretch, img_stretch))
        hist = np.hstack((img_gray_debug, hist_1, img_stretch_debug, hist_2))
        _, img_threshold = cv2.threshold(img_stretch, val_threshold, 255, cv2.THRESH_BINARY)

        return img_threshold, val_stretch_high, hist

    def _fn_find_lane(self, img_trans, img_threshold):
        #TODO : 수직 방향으로 구간 분할하기 위한 값들
        val_threshold_h, val_threshold_w = img_threshold.shape[:2]
        val_section_h = 4
        num_of_section = int(val_threshold_h / val_section_h)

        #TODO : 리스트 초기값 생성
        self._fn_init_list_pts()
        list_cx_left = [self.cx_left_pre] * 5
        list_cx_right = [self.cx_right_pre] * 5
        list_gap = [self.gap_pre] * 5

        #TODO : 수직 방향으로 구간 분할하고 각 구간마다 컨투어이용해서 차선영역 검출 및 분리 (from 5)
        for n_section in range(5, num_of_section):
            section_top = val_threshold_h - (n_section + 1) * val_section_h
            img_section = img_threshold[section_top:section_top + val_section_h, :]
            list_section = []

            cy = int(section_top + val_section_h / 2)
            if np.count_nonzero(img_section) > 10:
                _, list_contour, _ = cv2.findContours(img_section, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                for i in range(len(list_contour)):
                    contour = list_contour[i]
                    moment = cv2.moments(contour)
                    if moment['m00'] > 10:
                        xmid = int(moment['m10'] / moment['m00'])
                        list_section.append(xmid)
                        cv2.circle(img_trans, (xmid, cy), 4, (120, 0, 0), 1)

            if len(self.list_left_pts_x) >= 5:
                val_left_min_limit = 20
            elif len(self.list_right_pts_x) >= 5:
                val_left_min_limit = 40
            else:
                val_left_min_limit = 110

            if len(self.list_right_pts_x) >= 5:
                val_right_min_limit = 20
            elif len(self.list_left_pts_x) >= 5:
                val_right_min_limit = 40
            else:
                val_right_min_limit = 110

            if len(list_section) >= 2:
                list_left_gap = []
                list_right_gap = []
                for xmid in list_section:
                    list_left_gap.append(abs(list_cx_left[-1] - xmid))
                    list_right_gap.append(abs(list_cx_right[-1] - xmid))

                list_left_right_gap = list_left_gap + list_right_gap
                idx_left_right = list_left_right_gap.index(min(list_left_right_gap))

                if idx_left_right < len(list_section):
                    val_left_min = min(list_left_right_gap)
                    idx_left = idx_left_right
                    list_right_gap[idx_left] = 10000
                    idx_right = list_right_gap.index(min(list_right_gap))
                    val_right_min = min(list_right_gap)

                else:
                    val_right_min = min(list_left_right_gap)
                    idx_right = idx_left_right - len(list_section)
                    list_left_gap[idx_right] = 10000
                    idx_left = list_left_gap.index(min(list_left_gap))
                    val_left_min = min(list_left_gap)

                if val_left_min < val_left_min_limit and val_right_min < val_right_min_limit:
                    cx_left = list_section[idx_left]
                    cx_right = list_section[idx_right]

                    list_cx_left.append(cx_left)
                    list_cx_right.append(cx_right)
                    list_gap.append(cx_right - cx_left)

                    self.list_left_pts_x.append(cx_left)
                    self.list_left_pts_y.append(cy)
                    self.list_right_pts_x.append(cx_right)
                    self.list_right_pts_y.append(cy)
                    cv2.circle(img_trans, (cx_left, cy), 4, (0, 0, 255), 1)
                    cv2.circle(img_trans, (cx_right, cy), 4, (0, 255, 0), 1)

                elif val_left_min < val_left_min_limit:
                    cx = list_section[idx_left]
                    list_cx_left.append(cx)
                    list_cx_right.append(cx + list_gap[-1])
                    list_gap.append(list_gap[-1])

                    self.list_left_pts_x.append(cx)
                    self.list_left_pts_y.append(cy)
                    cv2.circle(img_trans, (cx, cy), 4, (0, 0, 255), 1)

                elif val_right_min < val_left_min_limit:
                    cx = list_section[idx_right]
                    list_cx_right.append(cx)
                    list_cx_left.append(cx - list_gap[-1])
                    list_gap.append(list_gap[-1])

                    self.list_right_pts_x.append(cx)
                    self.list_right_pts_y.append(cy)
                    cv2.circle(img_trans, (cx, cy), 4, (0, 255, 0), 1)

                else:
                    list_cx_left.append(list_cx_left[-1] + int((list_cx_left[-1] - list_cx_left[-5]) / 4))
                    list_cx_right.append(list_cx_right[-1] + int((list_cx_right[-1] - list_cx_right[-5]) / 4))
                    list_gap.append(list_cx_right[-1] - list_cx_left[-1])

            elif len(list_section) == 1:
                [cx] = list_section
                left_gap = abs(list_cx_left[-1] - cx)
                right_gap = abs(list_cx_right[-1] - cx)

                if left_gap < right_gap:
                    if left_gap < val_left_min_limit:
                        list_cx_left.append(cx)
                        list_cx_right.append(cx + list_gap[-1])
                        list_gap.append(list_gap[-1])

                        self.list_left_pts_x.append(cx)
                        self.list_left_pts_y.append(cy)
                        cv2.circle(img_trans, (cx, cy), 4, (0, 0, 255), 1)

                else:
                    if right_gap < val_right_min_limit:
                        list_cx_right.append(cx)
                        list_cx_left.append(cx - list_gap[-1])
                        list_gap.append(list_gap[-1])

                        self.list_right_pts_x.append(cx)
                        self.list_right_pts_y.append(cy)
                        cv2.circle(img_trans, (cx, cy), 4, (0, 255, 0), 1)

            else:
                list_cx_left.append(list_cx_left[-1] + int((list_cx_left[-1] - list_cx_left[-5]) / 4))
                list_cx_right.append(list_cx_right[-1] + int((list_cx_right[-1] - list_cx_right[-5]) / 4))
                list_gap.append(list_cx_right[-1] - list_cx_left[-1])

    def fn_lane_process(self, img_trans, img_threshold, right_lane=True, left_lane=True): # 외부 호출
        num_pit_lane = 30

        #TODO : morphology 연산으로 노이즈 제거
        img_threshold = cv2.morphologyEx(img_threshold, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        img_threshold = cv2.morphologyEx(img_threshold, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        val_threshold_h, val_threshold_w = img_threshold.shape[:2]

        #TODO : 양쪽 라인을 찾는 함수
        self._fn_find_lane(img_trans, img_threshold)

        #TODO : 라인 찾기 플래그
        if len(self.list_left_pts_x) > num_pit_lane:
            flag_find_left_lane = True
        else:
            flag_find_left_lane = False

        if len(self.list_right_pts_x) > num_pit_lane:
            flag_find_right_lane = True
        else:
            flag_find_right_lane = False

        #TODO : 차선 없애기
        if not right_lane:
            self.list_right_pts_x = []
            self.list_right_pts_y = []
        if not left_lane:
            self.list_left_pts_x = []
            self.list_left_pts_y = []

        arr_x = None
        arr_y = np.linspace(0, val_threshold_h - 1, val_threshold_h)

        #TODO : 양쪽 차선 모두 검출됬을 때 가운데 선 추출
        if len(self.list_left_pts_x) > num_pit_lane and len(self.list_right_pts_x) > num_pit_lane:
            try:
                list_left_lane_fit = np.polyfit(self.list_left_pts_y, self.list_left_pts_x, 2)
                arr_left_x = list_left_lane_fit[0] * arr_y ** 2 + list_left_lane_fit[1] * arr_y + list_left_lane_fit[2]
                arr_pts = np.array([np.transpose(np.vstack([arr_left_x, arr_y]))])
                cv2.polylines(img_trans, np.int_([arr_pts]), isClosed=False, color=(255, 0, 255), thickness=3)

                list_right_lane_fit = np.polyfit(self.list_right_pts_y, self.list_right_pts_x, 2)
                arr_right_x = list_right_lane_fit[0] * arr_y ** 2 + list_right_lane_fit[1] * arr_y + list_right_lane_fit[2]
                arr_pts = np.array([np.transpose(np.vstack([arr_right_x, arr_y]))])
                cv2.polylines(img_trans, np.int_([arr_pts]), isClosed=False, color=(255, 255, 0), thickness=3)

                arr_x = np.mean([arr_left_x, arr_right_x], axis=0)
                arr_pts = np.array([np.transpose(np.vstack([arr_x, arr_y]))])
                cv2.polylines(img_trans, np.int_([arr_pts]), isClosed=False, color=(255, 0, 0), thickness=3)

                self.shift_pre = int(abs(arr_right_x[self._center_y] - arr_x[self._center_y]))
                self.cx_left_pre = int(arr_left_x[self._center_y])
                self.cx_right_pre = int(arr_right_x[self._center_y])
                self.gap_pre = self.shift_pre * 2

            except Exception as e:
                rospy.logerr("Fail.both : " + e)

        #TODO : 왼쪽 차선만 검출됬을 때 shift
        elif len(self.list_left_pts_x) > num_pit_lane and len(self.list_right_pts_x) <= num_pit_lane:
            try:
                list_left_lane_fit = np.polyfit(self.list_left_pts_y, self.list_left_pts_x, 2)
                arr_left_x = list_left_lane_fit[0] * arr_y ** 2 + list_left_lane_fit[1] * arr_y + list_left_lane_fit[2]
                arr_pts = np.array([np.transpose(np.vstack([arr_left_x, arr_y]))])
                cv2.polylines(img_trans, np.int_([arr_pts]), isClosed=False, color=(255, 0, 255), thickness=3)

                arr_x = np.add(arr_left_x, self.shift_pre)
                arr_pts = np.array([np.transpose(np.vstack([arr_x, arr_y]))])
                cv2.polylines(img_trans, np.int_([arr_pts]), isClosed=False, color=(255, 0, 0), thickness=3)

                self.cx_left_pre = int(arr_left_x[self._center_y])
                self.cx_right_pre = int(arr_left_x[self._center_y] + self.shift_pre * 2)
                self.gap_pre = self.shift_pre * 2

            except Exception as e:
                rospy.logerr("Fail.left : " + e)

        #TODO : 오른쪽 차선만 검출됬을 때 shift
        elif len(self.list_left_pts_x) <= num_pit_lane and len(self.list_right_pts_x) > num_pit_lane:
            try:
                list_right_lane_fit = np.polyfit(self.list_right_pts_y, self.list_right_pts_x, 2)
                arr_right_x = list_right_lane_fit[0] * arr_y ** 2 + list_right_lane_fit[1] * arr_y + list_right_lane_fit[2]
                arr_pts = np.array([np.transpose(np.vstack([arr_right_x, arr_y]))])
                cv2.polylines(img_trans, np.int_([arr_pts]), isClosed=False, color=(255, 255, 0), thickness=3)

                arr_x = np.subtract(arr_right_x, self.shift_pre)
                arr_pts = np.array([np.transpose(np.vstack([arr_x, arr_y]))])
                cv2.polylines(img_trans, np.int_([arr_pts]), isClosed=False, color=(255, 0, 0), thickness=3)

                self.cx_right_pre = int(arr_right_x[self._center_y])
                self.cx_left_pre = int(arr_right_x[self._center_y] - self.shift_pre * 2)
                self.gap_pre = self.shift_pre * 2

            except Exception as e:
                rospy.logerr("Fail.right : " + e)

        #TODO : 중앙선 만들었을 경우
        if arr_x is not None:
            self._fn_limit_parameter()

            err = arr_x[self._center_y] - self._center_x
            self.error_pre = err
            rospy.loginfo("Error : " + str(err))

            cv2.circle(img_trans, (self._center_x, self._center_y), 8, (255, 255, 0), 2)
            cv2.circle(img_trans, (int(arr_x[self._center_y]), self._center_y), 8, (255, 0, 0), 2)

        #TODO : lane not detect 발생
        else:
            #rospy.logwarn("lane not found")
            err = self.error_pre

        #TODO : 리턴 값
        img_threshold_3ch = cv2.merge([img_threshold, img_threshold, img_threshold])
        img_debug = np.hstack((img_threshold_3ch, img_trans))

        return err, [flag_find_left_lane, flag_find_right_lane], img_debug

if __name__ == '__main__':
    lane = LaneProcessor()
