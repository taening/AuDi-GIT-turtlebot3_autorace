#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import math


class IntersectionDetector:
    def __init__(self):
        self.lower_blue = np.array([85, 90, 120], np.uint8)
        self.upper_blue = np.array([115, 255, 255], np.uint8)

    def fn_find_intersection_line(self, img_trans):
        # ROI 영역에 맞게 자른 이미지
        pers_height, pers_width = img_trans.shape[:2]  # shape is w384 x h240
        img_gray = cv2.cvtColor(img_trans[:int(pers_height * 1/ 2), :].copy(), cv2.COLOR_RGB2GRAY)
        _, img_intersection = cv2.threshold(img_gray, 180, 255, 0)
        img_intersection = cv2.morphologyEx(img_intersection, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        img_intersection = cv2.morphologyEx(img_intersection, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        img_debug = cv2.merge((img_intersection, img_intersection, img_intersection)).copy()

        _, list_intersection_contour, _ = cv2.findContours(img_intersection, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        intersection_check = False

        for intersection_contour in list_intersection_contour:
            cv2.drawContours(img_debug, [intersection_contour], 0, (0, 0, 255), 2)
            x_stop, y_stop, w_stop, h_stop = cv2.boundingRect(intersection_contour)
            cv2.putText(img_debug, 'w: {}, h: {}'.format(w_stop, h_stop), (intersection_contour[0][0][0]+10, intersection_contour[0][0][1]+10), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (0, 255, 255))
            if 330 < w_stop:
                cv2.drawContours(img_debug, [intersection_contour], 0, (0, 255, 0), 2)
                intersection_check = True

        return intersection_check, img_debug

    def fn_find_exit_line(self, img_trans, direction='left'):
        # ROI 영역에 맞게 자른 이미지
        pers_height, pers_width = img_trans.shape[:2]  # shape is w384 x h240
        if direction == 'left':
            img_gray = cv2.cvtColor(img_trans[:, int(pers_width * 1/ 2):].copy(), cv2.COLOR_RGB2GRAY)
        else:
            img_gray = cv2.cvtColor(img_trans[:, :int(pers_width * 1/ 2)].copy(), cv2.COLOR_RGB2GRAY)
        _, img_exit = cv2.threshold(img_gray, 190, 255, 0)
        img_exit = cv2.morphologyEx(img_exit, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        img_exit = cv2.morphologyEx(img_exit, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        img_debug = cv2.merge((img_exit, img_exit, img_exit)).copy()

        _, list_exit_contour, _ = cv2.findContours(img_exit, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        exit_check = False
        exit_pos = (0, 0)

        for exit_contour in list_exit_contour:
            cv2.drawContours(img_debug, [exit_contour], 0, (0, 0, 255), 2)
            x_exit, y_exit, w_exit, h_exit = cv2.boundingRect(exit_contour)
            bottom_most_pos = tuple(exit_contour[exit_contour[:, :, 1].argmax()][0])
            val_height = h_exit
            for pos_y in range(pers_height-1, 0, -1):
                if img_gray[pos_y, bottom_most_pos[0]] != 0:
                    val_height = pos_y
                    break

            cv2.putText(img_debug, 'w: {}, h: {}, length: {}'.format(w_exit, h_exit, val_height), (exit_contour[0][0][0]+10, exit_contour[0][0][1]+10), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (0, 255, 255))

            if h_exit > val_height * 4/5 and h_exit > pers_height/2:
                cv2.drawContours(img_debug, [exit_contour], 0, (0, 255, 0), 2)
                exit_pos = exit_contour[0][0]
                exit_check = True

        return exit_check, exit_pos, img_debug

    def fn_find_direction_sign(self, img_ori):
        left_sign_detect = False
        right_sign_detect = False

        img_height, img_width = img_ori.shape[:2]
        img_roi = img_ori[:int(img_height*1 / 2), :].copy()
        img_hsv = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)

        # Hsv fillter - Blue color
        img_mask_b = cv2.inRange(img_hsv, self.lower_blue, self.upper_blue)
        img_mask_b = cv2.morphologyEx(img_mask_b, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
        img_mask_b = cv2.morphologyEx(img_mask_b, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        #_, list_obj_contour, _ = cv2.findContours(img_mask_b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _, list_obj_contour, _ = cv2.findContours(img_mask_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        img_blue = cv2.bitwise_and(img_roi, img_roi, mask=img_mask_b)
        img_debug = img_roi.copy()

        list_obj = []

        for obj_contour in list_obj_contour:
            #cv2.drawContours(img_blue, [contour], 0, (0, 0, 255), 2)
            x, y, w, h = cv2.boundingRect(obj_contour)
            area = cv2.contourArea(obj_contour)
            aspect_ratio = float(w) / h
            area_ratio = float(area) / (w*h)
            cv2.rectangle(img_debug, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img_debug, 'w: {}, h: {}, aspect_ratio: {:.2f}, area_ratio: {:.2f}'.format(w, h, aspect_ratio, area_ratio), (x+10, y+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 127, 0))

            if (50 < w < 150) and (50 < h < 150) and (0.8 < aspect_ratio < 2.5) and (area_ratio > 0.5):
                cv2.rectangle(img_debug, (x, y), (x + w, y + h), (0, 255, 255), 2)
                list_obj.append((img_roi[y:y+h, x:x+w].copy(), (x, y, w, h)))

        for (img_obj, (obj_x, obj_y, obj_w, obj_h)) in list_obj:
            img_obj_gray = cv2.cvtColor(img_obj, cv2.COLOR_BGR2GRAY)
            _, img_obj_binary = cv2.threshold(img_obj_gray, 180, 255, cv2.THRESH_BINARY)
            img_obj_binary = cv2.morphologyEx(img_obj_binary, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            _, list_arrow_contour, _ = cv2.findContours(img_obj_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            obj_x_mid = int(obj_w / 2)
            obj_y_mid = int(obj_h / 2)

            min_val_dis = 30
            bottom_most_pos = None

            for arrow_contour in list_arrow_contour:
                mask_arrow = np.zeros(img_obj_gray.shape, np.uint8)
                cv2.drawContours(mask_arrow, [arrow_contour], 0, 255, -1)
                arrow_x, arrow_y, arrow_w, arrow_h = cv2.boundingRect(arrow_contour)
                cv2.rectangle(img_debug, (obj_x + arrow_x, obj_y + arrow_y), (obj_x + arrow_x + arrow_w, arrow_y + obj_y + arrow_h), (255, 255, 0), 1)
                arrow_area = cv2.contourArea(arrow_contour)
                arrow_aspect_ratio = float(arrow_w) / arrow_h
                arrow_area_ratio = float(arrow_area) / (arrow_w * arrow_h)

                arrow_x_mid = int(arrow_x + arrow_w / 2)
                arrow_y_mid = int(arrow_y + arrow_h / 2)

                if (0.4 * obj_w < arrow_w) and (0.4 * obj_h < arrow_h) and (0.5 < arrow_aspect_ratio < 2) and (arrow_area_ratio > 0.3):
                    val_dis = math.sqrt((arrow_x_mid - obj_x_mid) ** 2 + (arrow_y_mid - obj_y_mid) ** 2)
                    if val_dis < min_val_dis:
                        min_val_dis = val_dis

                        #left_most_pos = tuple(obj_contour[obj_contour[:, :, 0].argmin()][0])
                        #right_most_pos = tuple(obj_contour[obj_contour[:, :, 0].argmax()][0])
                        #top_most_pos = tuple(obj_contour[obj_contour[:, :, 1].argmin()][0])
                        bottom_most_pos = tuple(arrow_contour[arrow_contour[:, :, 1].argmax()][0])

            if bottom_most_pos is not None:
                cv2.circle(img_debug, (obj_x + bottom_most_pos[0], obj_y + bottom_most_pos[1]), 4, (0, 0, 255), -1)
                cv2.line(img_debug, (obj_x + obj_x_mid, obj_y), (obj_x + obj_x_mid, obj_y + obj_h), (255, 0, 255), 2)
                if bottom_most_pos[0] > obj_x_mid:
                    left_sign_detect = True
                    cv2.putText(img_debug, 'LEFT', (obj_x+10, obj_y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
                    cv2.rectangle(img_debug, (obj_x, obj_y), (obj_x + obj_w, obj_y + obj_h), (255, 0, 0), 2)
                else:
                    right_sign_detect = True
                    cv2.putText(img_debug, 'RIGHT', (obj_x+3, obj_y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
                    cv2.rectangle(img_debug, (obj_x, obj_y), (obj_x + obj_w, obj_y + obj_h), (0, 255, 0), 2)

        return left_sign_detect, right_sign_detect, np.vstack((img_debug, img_blue))
