#!/usr/bin/env python
# -*-coding:utf-8-*-
from sensor_msgs.msg import Image, CompressedImage
from lane_processor import LaneProcessor
from cv_bridge import CvBridge
import numpy as np
import cv2
import rospy


class PerspectiveParameterNode:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/image_raw/compressed', CompressedImage, self.image_callback)
        self.bridge = CvBridge()

        self.lp = LaneProcessor()

        self.height, self.width = 0, 0
        self.ratio_w, self.ratio_h = 300, 250
        self.list_pos_pre, self.list_pos = [], []
        self.center_x = 192
        self.center_y = 190 #given

    def image_callback(self, msg):
        #img_ori = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        np_arr = np.fromstring(msg.data, np.uint8)
        img_ori = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img_ori = self.lp.fn_lane_roi(img_ori)
        self.list_pos_ori, self.list_pos_trans = self.perspective_transform(img_ori)
        try:
            self.list_pos_ori, self.list_pos_trans = self.perspective_transform(img_ori)
            #rospy.loginfo(self.list_pos_ori)
            #rospy.loginfo(self.list_pos_trans)

            val_ori_height, val_ori_width = img_ori.shape[:2]
            val_trans_width = int(val_ori_height * (self.ratio_w / self.ratio_h) * 1.8)
            val_trans_height = val_ori_height
            img_trans = LaneProcessor.trans_perspective(img_ori, self.list_pos_ori, self.list_pos_trans, val_trans_width, val_trans_height)
            #mask_line = LaneProcessor.hsv_filter(img_trans)
            mask_line, _ = self.lp.fn_lane_threshold(img_trans)
            img_line = cv2.bitwise_and(img_trans, img_trans, mask=mask_line)

            img_gray = cv2.cvtColor(img_line, cv2.COLOR_BGR2GRAY)
            _, img_threshold = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY)
            img_threshold = cv2.morphologyEx(img_threshold, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            img_threshold = cv2.morphologyEx(img_threshold, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

            #cv2.imshow('7', img_ori)
            #cv2.imshow('5', img_trans)
            #cv2.imshow('6', img_threshold)
            #cv2.waitKey(3)

            self.center_x = self.get_center(img_threshold)

            rospy.loginfo("original_4point : " + str(self.list_pos_ori))
            rospy.loginfo("transform_4point : " + str(self.list_pos_trans))
            rospy.loginfo("transform_size : " + str([val_trans_width, val_trans_height]))
            rospy.loginfo("center_point : " + str([self.center_x, self.center_y]))
            rospy.set_param("/perspective/original", self.list_pos_pre)
            rospy.set_param("/perspective/trans", self.list_pos)
            rospy.set_param("/perspective/size", [val_trans_width, val_trans_height])
            rospy.set_param("/perspective/center", [self.center_x, self.center_y])
        except Exception as e:
            rospy.loginfo("Try again getting parameter : " + str(e))
        else:
            rospy.signal_shutdown("Success getting parameter!")

    def perspective_transform(self, image):
        img_line = LaneProcessor.hsv_filter(image)
        #img_line = cv2.morphologyEx(img_line, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        #img_line = cv2.morphologyEx(img_line, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        #img_line = cv2.bitwise_and(image, image, mask=mask_line)

        #cv2.imshow('1', image)
        #cv2.imshow('2', img_line)
        #cv2.waitKey(3)

        self.height, self.width = img_line.shape[:2]

        val_left_w0 = 0
        val_left_h0 = int(self.height / 4)
        img_left = img_line[val_left_h0: val_left_h0 + (int(self.height/2)),
                            val_left_w0: val_left_w0 + (int(self.width / 2))]

        val_right_w0 = int(self.width / 2)
        val_right_h0 = int(self.height / 4)
        img_right = img_line[val_right_h0: val_right_h0 + (int(self.height / 2)),
                             val_right_w0: val_right_w0 + (int(self.width/2))]

        img_left_edges = cv2.Canny(img_left, 80, 180)
        img_right_edges = cv2.Canny(img_right, 80, 180)

        #cv2.imshow('3', img_left_edges)
        #cv2.imshow('4', img_right_edges)
        #cv2.waitKey(3)

        list_hough_left = cv2.HoughLines(img_left_edges, 1, np.pi/180, 30)
        list_hough_right = cv2.HoughLines(img_right_edges, 1, np.pi/180, 30)

        list_left_x0 = [rho * np.cos(theta) for [[rho, theta]] in list_hough_left[:2]]
        idx_left = list_left_x0.index(max(list_left_x0))
        [[rho, theta]] = list_hough_left[idx_left]
        line_left = self.hough_pos(rho, theta, val_left_w0, val_left_h0, self.height)

        list_right_x0 = [rho * np.cos(theta) for [[rho, theta]] in list_hough_right[:2]]
        idx_right = list_right_x0.index(min(list_right_x0))
        [[rho, theta]] = list_hough_right[idx_right]
        line_right = self.hough_pos(rho, theta, val_right_w0, val_right_h0, self.height)

        val_ori_height, val_ori_width = image.shape[:2]
        x_mid = int(val_ori_height * (self.ratio_w / self.ratio_h) * 0.9)
        x_width = ((line_left[1][1] + line_right[1][1]) - (line_left[0][1] + line_right[0][1])) * self.ratio_w // (4 * self.ratio_h) * 2 / 2

        pos1_pre = [line_left[0][0], line_left[0][1]]
        pos1 = [x_mid - x_width, line_left[0][1]]
        pos2_pre = [line_left[1][0], line_left[1][1]]
        pos2 = [x_mid - x_width, line_left[1][1]]
        pos3_pre = [line_right[0][0], line_right[0][1]]
        pos3 = [x_mid + x_width, line_right[0][1]]
        pos4_pre = [line_right[1][0], line_right[1][1]]
        pos4 = [x_mid + x_width, line_right[1][1]]

        self.list_pos_pre = [pos1_pre, pos2_pre, pos3_pre, pos4_pre]
        self.list_pos = [pos1, pos2, pos3, pos4]
        return self.list_pos_pre, self.list_pos

    def get_center(self, image):
        cy = self.center_y
        #img_section = image[cy-10:cy+10, :]
        img_section = image[cy-30:cy+30, :]
        #cv2.imshow('8', img_section)

        if np.count_nonzero(img_section) > 15:
            moment = cv2.moments(img_section)
            if moment['m00'] > 0:
                xmid = int(moment['m10'] / moment['m00'])
            else:
                raise Exception('moment process error')
            img_section_left = img_section[:, :xmid]
            img_section_right = img_section[:, xmid:]

            if image[cy, xmid] != 255:
                if np.count_nonzero(img_section_left) > 25 and np.count_nonzero(img_section_right) > 25:
                    moment_left = cv2.moments(img_section_left)
                    if moment_left['m00'] > 0:
                        cx1 = int(moment_left['m10'] / moment_left['m00'])
                    else:
                        raise Exception('moment process error')

                    moment_right = cv2.moments(img_section_right)
                    if moment_right['m00'] > 0:
                        cx2 = int(xmid + moment_right['m10'] / moment_right['m00'])
                    else:
                        raise Exception('moment process error')

                    cx = int((cx1 + cx2) / 2)
                    return cx
                else:
                    raise Exception('left lane or right lane not pound')
            else:
                raise Exception('can\'t divide into two image')
        else:
            raise Exception('lane not pound in vertical section')

    @staticmethod
    def hough_pos(rho, theta, w0, h0, height):
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        x0 = w0 + c_theta*rho
        y0 = h0 + s_theta*rho
        r1 = y0 // c_theta
        r2 = (height - y0) // c_theta
        upper_x = int(x0 - r1 * (-s_theta))
        upper_y = int(y0 - r1 * c_theta)
        lower_x = int(x0 + r2 * (-s_theta))
        lower_y = int(y0 + r2 * c_theta)
        return [[upper_x, upper_y], [lower_x, lower_y]]

    @staticmethod
    def main():
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node("Perspective_Parameter_Node")
    node = PerspectiveParameterNode()
    node.main()
