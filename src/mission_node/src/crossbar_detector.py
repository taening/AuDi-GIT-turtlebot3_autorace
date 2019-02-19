#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import itertools
import cv2


class CrossbarDetector:
    def __init__(self):
        self.lower_red = np.array([160,105,68], np.uint8)
        self.upper_red = np.array([188,255, 255], np.uint8)
        self.kernel = np.ones((7, 7), np.uint8)  # 컨투어 이용할 번수

        self.difference_y_limit = 25
        self.difference_y_ratio = 0.5
        self.difference_x_limit = 50
        self.difference_x_ratio = 0.5
        self.intensity_gap_limit = 50
        self.error_width_limit = 25
        self.error_width_ratio = 0.5
        self.error_height_limit = 13
        self.error_height_ratio = 0.3

    def fn_find_crossbar(self, img_ori):
        crossbar_detect = False

        img_height, img_width = img_ori.shape[:2]
        img_roi = img_ori[ : int(img_height*1 / 2), :].copy()
        img_hsv = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)

        # Hsv fillter - Red color
        img_mask_r = cv2.inRange(img_hsv, self.lower_red, self.upper_red)
        img_mask_r = cv2.morphologyEx(img_mask_r, cv2.MORPH_OPEN, self.kernel)
        img_red = cv2.bitwise_and(img_roi, img_roi, mask=img_mask_r)

        _, list_contour, _ = cv2.findContours(img_mask_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        list_all_obj_pos = []

        for contour in list_contour:
            cv2.drawContours(img_red, [contour], 0, (0, 0, 255), 2)
            x, y, w, h = cv2.boundingRect(contour)
            cv2.putText(img_red, 'w: {}, h: {}, aspect_ratio: {:.2f}'.format(w, h, (float(w) / h)), (x+2, y+h+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            if (10 < w < 80) and (10 < h < 80) and (0.5 < (float(w) / h) < 2):
                cv2.drawContours(img_red, [contour], 0, (0, 255, 0), 2)
                moment = cv2.moments(contour)
                if moment['m00'] > 10:
                    cx = int(moment['m10'] / moment['m00'])
                    cy = int(moment['m01'] / moment['m00'])
                    list_all_obj_pos.append([cx, cy, w, h])

        num_obj = 5
        if len(list_all_obj_pos) >= num_obj:
            list_combination_obj_pos = list(itertools.combinations(list_all_obj_pos, num_obj))

            for list_obj_pos in list_combination_obj_pos:
                flag_y = True
                flag_x = True
                flag_gap = True

                obj_idx = [x for x in range(len(list_obj_pos))]
                for ii in range(len(obj_idx)):
                    for jj in range(ii + 1, len(obj_idx)):
                        if list_obj_pos[obj_idx[ii]][0] > list_obj_pos[obj_idx[jj]][0]:
                            temp = obj_idx[jj]
                            obj_idx[jj] = obj_idx[ii]
                            obj_idx[ii] = temp

                for idx_y in range(num_obj):
                    difference_y = abs(list_obj_pos[obj_idx[2]][1] - list_obj_pos[obj_idx[idx_y]][1])
                    self.difference_y_limit = int(list_obj_pos[obj_idx[2]][3] * self.difference_y_ratio)
                    if difference_y > self.difference_y_limit:
                        flag_y = False

                difference_x = [0] * (num_obj-1)
                for idx_x in range(num_obj-1):
                    difference_x[idx_x] = abs(list_obj_pos[obj_idx[idx_x + 1]][0] - list_obj_pos[obj_idx[idx_x]][0])
                #difference_x_min = min(difference_x)
                #difference_x_max = max(difference_x)
                list_width = [0] * num_obj
                for idx_width in range(num_obj):
                    list_width[idx_width] = list_obj_pos[obj_idx[idx_width]][2]
                width_max = max(list_width)
                self.difference_x_limit = int(width_max * self.difference_x_ratio)
                for idx_x_mid in range(1, num_obj-1):
                    if abs(difference_x[idx_x_mid] - difference_x[idx_x_mid - 1]) > self.difference_x_limit:
                        flag_x = False

                #error_width = [0] * 5
                #for idx_width in range(5):
                #    error_width[idx_width] = list_obj_pos[obj_idx[idx_width]][2]
                #error_width_max = max(error_width)
                #error_width_min = min(error_width)
                #self.error_width_limit = int(error_width_max * self.error_width_ratio)
                #if error_width_max - error_width_min > self.error_width_limit:
                #    flag_width = False

                #error_height = [0] * 5
                #for idx_height in range(5):
                #    error_height[idx_height] = list_obj_pos[obj_idx[idx_height]][3]
                #error_height_max = max(error_height)
                #error_height_min = min(error_height)
                #self.error_height_limit = int(error_height_max * self.error_height_ratio)
                #if error_height_max - error_height_min > self.error_height_limit:
                #    flag_height = False

                img_gray = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
                for idx_gap in range(num_obj-1):
                    pos_left_x = list_obj_pos[obj_idx[idx_gap]][0]
                    pos_left_y = list_obj_pos[obj_idx[idx_gap]][1]
                    pos_right_x = list_obj_pos[obj_idx[idx_gap + 1]][0]
                    pos_right_y = list_obj_pos[obj_idx[idx_gap + 1]][1]
                    pos_gap_x = int((pos_left_x + pos_right_x) / 2)
                    pos_gap_y = int((pos_left_y + pos_right_y) / 2)
                    val_left = img_gray[pos_left_y, pos_left_x]
                    val_right = img_gray[pos_right_y, pos_right_x]
                    val_gap = img_gray[pos_gap_y, pos_gap_x]
                    if ((val_left + val_right) / 2) + self.intensity_gap_limit > val_gap:
                        flag_gap = False

                #print(list_obj_pos[obj_idx[0]])
                #print(list_obj_pos[obj_idx[1]])
                #print(list_obj_pos[obj_idx[2]])
                #print(list_obj_pos[obj_idx[3]])
                #print(list_obj_pos[obj_idx[4]])
                #print(difference_x)
                #print(flag_y, flag_x, flag_gap)
                if flag_y and flag_x and flag_gap:
                    for ii in range(0, num_obj):
                        cv2.circle(img_red, tuple(list_obj_pos[obj_idx[ii]][:2]), 8, (127, 0, 0), 4)
                    cv2.circle(img_red, tuple(list_obj_pos[obj_idx[(num_obj-1)//2]][:2]), 8, (255, 0, 0), 4)
                    for ii in range(1, num_obj-1):
                        cv2.putText(img_red, str(abs(difference_x[ii] - difference_x[ii-1])), tuple(list_obj_pos[obj_idx[ii]][:2]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                    for ii in range(0, num_obj-1):
                        cv2.circle(img_red, (int((list_obj_pos[obj_idx[ii]][0] + list_obj_pos[obj_idx[ii+1]][0]) / 2), int((list_obj_pos[obj_idx[ii]][1] + list_obj_pos[obj_idx[ii+1]][1]) / 2)), 4, (255, 255, 0), 2)
                    cv2.line(img_red, (0, list_obj_pos[obj_idx[(num_obj-1)//2]][1] - self.difference_y_limit), (img_red.shape[1], list_obj_pos[obj_idx[(num_obj-1)//2]][1] - self.difference_y_limit), (0, 127, 127), 3)
                    cv2.line(img_red, (0, list_obj_pos[obj_idx[(num_obj-1)//2]][1] + self.difference_y_limit), (img_red.shape[1], list_obj_pos[obj_idx[(num_obj-1)//2]][1] + self.difference_y_limit), (0, 127, 127), 3)
                    crossbar_detect = True
                    break

        return crossbar_detect, img_red

    def fn_confirm_crossbar(self, img_ori):
        crossbar_detect = False

        img_height, img_width = img_ori.shape[:2]
        img_roi = img_ori[ : int(img_height*1 / 2), :].copy()
        img_hsv = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)

        # Hsv fillter - Red color
        img_mask_r = cv2.inRange(img_hsv, self.lower_red, self.upper_red)
        img_mask_r = cv2.morphologyEx(img_mask_r, cv2.MORPH_OPEN, self.kernel)
        img_red = cv2.bitwise_and(img_roi, img_roi, mask=img_mask_r)

        _, list_contour, _ = cv2.findContours(img_mask_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in list_contour:
            cv2.drawContours(img_red, [contour], 0, (255, 0, 0), 2)
            x, y, w, h = cv2.boundingRect(contour)
            cv2.putText(img_red, 'w: {}, h: {}, aspect_ratio: {:.2f}'.format(w, h, (float(w) / h)), (x+2, y+h+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            if (50 < w < 140) and (50 < h < 140) and (0.5 < (float(w) / h) < 2):
                cv2.drawContours(img_red, [contour], 0, (0, 255, 0), 2)
                crossbar_detect = True

        return crossbar_detect, img_red
