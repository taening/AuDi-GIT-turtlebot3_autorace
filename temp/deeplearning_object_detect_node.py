#!/usr/bin/env python
# -*- coding: utf-8 -*-

# In[1]:

from std_msgs.msg import Float32, UInt8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import tensorflow as tf
import numpy as np
import rospy
import cv2
import copy
import threading
import time


class DeeplearningObjectDetectorNode:
    def __init__(self):
        self.sub_img_cam = rospy.Subscriber('/image_raw/compressed', CompressedImage, self.cb_image_receive, queue_size=1)

        self.pub_img_object = rospy.Publisher('/deeplearning/image/object', Image, queue_size=1)

        self.cvBridge = CvBridge()

        self.path = '/home/sim/deeplearning/object_model'
        self.learning_rate = 0.01
        self.training = tf.placeholder(tf.bool)
        
        self.X = tf.placeholder(tf.float32, [None, 48, 48, 3])
        self.Y = tf.placeholder(tf.float32, [None, 5])
        
        self.model, self.cost, self.optimizer = self.set_model()
        self.sess = tf.Session()
        self.sess.run(tf.global_variables_initializer())

        # HSV Range 관련 변수 (H:색상, S:채도, V:명도)
        self.lower = {'blue': np.array([80, 80, 70]),
                      'yellow': np.array([0, 50, 70]),
                      'red': np.array([170, 50, 100])}
        self.upper = {'blue': np.array([120, 255, 255]),
                      'yellow': np.array([30, 255, 255]),
                      'red': np.array([180, 255, 255])}

        self.detect_time_pre = time.time()

    def cb_image_receive(self, msg):
        detect_time_now = time.time()
        if detect_time_now < self.detect_time_pre + 0.2:
            return
        self.detect_time_pre = detect_time_now

        process_time_pre = time.time()
        np_arr = np.fromstring(msg.data, np.uint8)
        img_ori = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img_object = img_ori
        #self.img_ori_global = img_ori

        dict_detect_name = {0:'parking', 1:'tunnel', 2:'crossbar', 3:'none', 4:'crosswalk'}
        dict_detect_color = {0:(255, 0, 0), 1:(0, 255, 255), 2:(0, 0, 255), 3:(0, 0, 0), 4:(255, 255, 0)}

        list_info_roi = self.rectangle_detector(img_ori)
        for img_roi, pts1, pts2 in list_info_roi:
            #print(img_roi.shape)
            object_num = self.predict([img_roi])[0]
            print(object_num)
            object_name = dict_detect_name[object_num]
            print('detect : ' + object_name)
            object_color = dict_detect_color[object_num]
            cv2.rectangle(img_object, pts1, pts2, object_color, 2)
            cv2.putText(img_object, object_name, pts1, cv2.FONT_ITALIC, 1, object_color, 2)

        self.pub_img_object.publish(self.cvBridge.cv2_to_imgmsg(img_object, "bgr8"))
        cv2.imshow('kiki', img_object)
        cv2.waitKey(1)

        process_time_now = time.time()
        rospy.loginfo('process time: ' + str(process_time_now - process_time_pre))

    def rectangle_detector(self, frame):
        process = list()
        list_roi = list()

        # Original 저장
        ori = copy.deepcopy(frame)
        process.append(ori)

        # Gaussian Blur 변환
        frame = cv2.GaussianBlur(frame, (5, 5), 3)
        process.append(frame)

        # Convert BGR to HSV && Search ROI Area
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red = cv2.inRange(frame, self.lower['red'], self.upper['red'])
        yellow = cv2.inRange(frame, self.lower['yellow'], self.upper['yellow'])
        blue = cv2.inRange(frame, self.lower['blue'], self.upper['blue'])
        frame = cv2.bitwise_or(red, yellow)
        frame = cv2.bitwise_or(frame, blue)
        process.append(frame)

        # Morphology 변환
        kernel = np.ones((3, 3), np.int8)
        frame = cv2.dilate(frame, kernel, iterations=2)
        frame = cv2.erode(frame, kernel, iterations=1)
        process.append(frame)

        # Contour 변환 && Area info 저장
        _, contours, _ = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        info = [[np.min(contour, 0).ravel(), np.max(contour, 0).ravel()] for contour in contours]

        tmp = copy.deepcopy(info)
        count = 0
        for i in info:
            w, h = i[1] - i[0]
            if (0.8 * h < w < 1.2 * h) and (40 < w < 200 and 40 < h < 200):
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
                roi = cv2.resize(roi, (48, 48), cv2.INTER_CUBIC)
                list_roi.append([roi, tuple(info[i][0]), tuple(info[i][1])])
                frame = cv2.rectangle(ori, tuple(info[i][0]), tuple(info[i][1]), (0, 255, 0), 3)
        process.append(frame)

        return list_roi
        
    def set_model(self):
        '''
        W1 = tf.Variable(tf.random_normal([3, 3, 3, 16], stddev= 0.01))
        L1 = tf.nn.conv2d(self.X, W1, strides=[1, 1, 1, 1], padding='SAME')
        L1 = tf.nn.relu(L1)
        L1 = tf.nn.max_pool(L1, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
        
        W2 = tf.Variable(tf.random_normal([3, 3, 16, 32], stddev=0.01))
        L2 = tf.nn.conv2d(L1, W2, strides=[1, 1, 1, 1], padding='SAME')
        L2 = tf.nn.relu(L2)
        L2 = tf.nn.max_pool(L2, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
        
        W3 = tf.Variable(tf.random_normal([3, 3, 32, 64], stddev=0.01))
        L3 = tf.nn.conv2d(L2, W3, strides=[1, 1, 1, 1], padding='SAME')
        L3 = tf.nn.relu(L3)
        L3 = tf.nn.max_pool(L3, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
        
        W4 = tf.Variable(tf.random_normal([3, 3, 64, 128], stddev=0.01))
        L4 = tf.nn.conv2d(L3, W4, strides=[1, 1, 1, 1], padding='SAME')
        L4 = tf.nn.relu(L4)
        
        W5 = tf.Variable(tf.random_normal([6 * 6 * 128, 256], stddev=0.01))
        L5 = tf.reshape(L4, [-1, 6 * 6 * 128])
        L5 = tf.matmul(L5, W5)
        L5 = tf.nn.relu(L5)
        L5 = tf.contrib.layers.batch_norm(L5, center=True, scale=True, is_training='phase', scope='bn')
        
        W6 = tf.Variable(tf.random_normal([256, 5], stddev=0.01))
        model = tf.matmul(L5, W6)
        
        cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits_v2(logits=model, labels=self.Y))
        optimizer = tf.train.AdamOptimizer(self.learning_rate).minimize(cost)
        '''

        with tf.name_scope('Layer1'):
            L1 = tf.layers.conv2d(self.X, 16, [3, 3], padding='SAME', activation=tf.nn.relu)
            L1 = tf.layers.max_pooling2d(L1, [2, 2], [2, 2])

        with tf.name_scope('Layer2'):
            L2 = tf.layers.conv2d(L1, 32, [3, 3], padding='SAME', activation=tf.nn.relu)
            L2 = tf.layers.max_pooling2d(L2, [2, 2], [2, 2])

        with tf.name_scope('Layer3'):
            L3 = tf.layers.conv2d(L2, 64, [3, 3], padding='SAME', activation=tf.nn.relu)
            L3 = tf.layers.max_pooling2d(L3, [2, 2], [2, 2])

        with tf.name_scope('Layer4'):
            L4 = tf.layers.conv2d(L3, 128, [3, 3], padding='SAME', activation=tf.nn.relu)

        with tf.name_scope('Layer5'):
            L5 = tf.layers.conv2d(L4, 128, [3, 3], padding='SAME', activation=tf.nn.relu)
            L5 = tf.contrib.layers.flatten(L5)
            L5 = tf.layers.dense(L5, 256, activation=tf.nn.relu)
            L5 = tf.contrib.layers.batch_norm(L5, center=True, scale=True, is_training='phase', scope='bn')

        with tf.name_scope('Output'):
            model = tf.layers.dense(L5, 5, activation=None)

        with tf.name_scope('CostFunction'):
            cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits_v2(logits=model, labels=self.Y))
            optimizer = tf.train.AdamOptimizer(self.learning_rate).minimize(cost)

        return model, cost, optimizer
    
    def train(self, x_data, y_data):
        saver = tf.train.Saver()

        cost_val, _ = self.sess.run([self.cost, self.optimizer], feed_dict={self.X: x_data, self.Y: y_data})
        for epoch in range(40):
            cost_val = self.sess.run([self.cost, self.optimizer], feed_dict={self.X: x_data, self.Y: y_data})
            print('Epoch:', str(epoch + 1), ', cost=', str(cost_val))
        saver.save(self.sess, '/Users/seopaul/ipynb/model/cnn.ckpt')

    def predict(self, x_data):
        prediction = tf.argmax(self.model, 1)
        return self.sess.run(prediction, feed_dict={self.X: x_data})

    def accuracy(self, x_data, y_data):
        prediction = tf.argmax(self.model, 1)
        target = tf.argmax(self.Y, 1)

        is_correct = tf.equal(prediction, target)
        accuracy = tf.reduce_mean(tf.cast(is_correct, tf.float32))
        
        predict, answer = self.sess.run([prediction, target], feed_dict={self.X: x_data, self.Y: y_data})
        print("Prediction : " + str(predict) + "\nAnswer : " + str(answer))
        
        ac = self.sess.run(accuracy, feed_dict={self.X: x_data, self.Y: y_data})
        print("Accuracy : " + str(ac))
    
    def restore(self):
        saver = tf.train.Saver()
        saver.restore(self.sess, self.path + '/cnn.ckpt')

    @staticmethod
    def main():
        rospy.spin()

# In[2]:

if __name__=='__main__':
    rospy.init_node('Deeplearning_Object_Detect_Node')
    obj = DeeplearningObjectDetectorNode()
    obj.restore()
    obj.main()
