#!/usr/bin/env python
# -*-coding:utf-8-*-
import numpy as np
import cv2
import copy


class RectangleDetector:
    def __init__(self):
        self.process = list()
        self.roi = list()

        # HSV Range 관련 변수 (H:색상, S:채도, V:명도)
        self.lower = {'blue': np.array([85, 100, 120]),
                      'yellow': np.array([0, 50, 70]),
                      'red': np.array([170, 50, 100])}
        self.upper = {'blue': np.array([115, 255, 255]),
                      'yellow': np.array([30, 255, 255]),
                      'red': np.array([180, 255, 255])}

    def clear(self):
        self.process = []
        self.roi = []

    def __call__(self, frame):
        #frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_CUBIC)
        # Original 저장
        ori = copy.deepcopy(frame)
        self.process.append(ori)

        # Gaussian Blur 변환
        frame = cv2.GaussianBlur(frame, (5, 5), 3)
        self.process.append(frame)

        # Convert BGR to HSV && Search ROI Area
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # red = cv2.inRange(frame, self.lower['red'], self.upper['red'])
        # yellow = cv2.inRange(frame, self.lower['yellow'], self.upper['yellow'])
        frame = cv2.inRange(frame, self.lower['blue'], self.upper['blue'])
        # frame = cv2.bitwise_or(red, yellow)
        # frame = cv2.bitwise_or(frame, blue)
        self.process.append(frame)

        # Morphology 변환
        #kernel = np.ones((3, 3), np.uint8)
        frame = cv2.dilate(frame, np.ones((15, 15), np.uint8), iterations=1)
        frame = cv2.erode(frame, np.ones((3, 3), np.uint8), iterations=1)
        self.process.append(frame)
        img_debug = cv2.bitwise_and(ori, ori, mask=frame)

        # Contour 변환 && Area info 저장
        _, contours, _ = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        info = [[np.min(contour, 0).ravel(), np.max(contour, 0).ravel()] for contour in contours]

        tmp = copy.deepcopy(info)
        count = 0
        for i in info:
            w, h = i[1] - i[0]
            if (0.75 * h < w < 2.5 * h) and (80 < w < 200 and 40 < h < 200):
                count = count + 1
            else:
                del(tmp[count])
        info = tmp

        # ROI Areas 추출
        if len(info) == 0:
            frame = ori
        else:
            for i in range(len(info)):
                roi = ori[info[i][0][1]: info[i][1][1], info[i][0][0]: info[i][1][0], :]
                roi = cv2.resize(roi, (80, 80), cv2.INTER_CUBIC)
                self.roi.append((roi, ((info[i][0][0], info[i][0][1]), (info[i][1][0], info[i][1][1]))))
        self.process.append(frame)

        return self.roi, img_debug


if __name__ == '__main__':
    rect = RectangleDetector()
    count = 1
    for i in range(0, 2882):
        # 건널목 : 810, 주차 : 1038, 정지: 975, 터널: 1007
        img = cv2.imread("/Volumes/Transcend/PycharmProjects/RectangleDetect/data/" + str(i) + ".jpg")
        result = rect(img)

        if len(result) == 1:
            cv2.imwrite("/Volumes/Transcend/PycharmProjects/RectangleDetect/label/" + str(count) + ".jpg", result[0])
            count += 1
        else:
            for j in range(len(result)):
                cv2.imwrite("/Volumes/Transcend/PycharmProjects/RectangleDetect/label/" + str(count) + ".jpg", result[j])
                count += 1

        rect.process.clear()
        rect.roi.clear()
