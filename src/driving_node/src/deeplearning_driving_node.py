#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import Float32, UInt8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import tensorflow as tf
import numpy as np
import enum
import rospy
import cv2
import threading
import time

# Hyper Prarameters
training_epochs = 10
batch_size = 100
learning_rate = 0.0001
img_height = 64
img_width = 120
img_channel = 3
total_data = 50000


class DeeplearningDrivingNode:
    def __init__(self, sess, name):
        self.cvBridge = CvBridge()

        self.sess = sess
        self.name = name
        self._build_net()

        self.driving_mode_step = enum.Enum('step_of_driving_mode', 'manual_mode lane_mode right_lane_mode left_lane_mode intensity_lane_mode deeplearning_lane_mode tunnel_mode spectate_mode')
        self.driving_mode = self.driving_mode_step.manual_mode.value

        self.sub_driving_mode = rospy.Subscriber('/mission/mod/driving', UInt8, self.cb_driving_mode, queue_size=1)

        self.sub_img_rev = rospy.Subscriber('/controller/image/driving', CompressedImage, self.cb_image_receive, queue_size=1)

        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.on_shutdown(self.fn_stop)

    def fn_driving_deeplearning(self, linear_vel, angular_vel):
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.pub_vel.publish(twist_msg)

    def cb_image_receive(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        img_ori = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if self.driving_mode == self.driving_mode_step.deeplearning_lane_mode.value:
            driving_time_pre = time.time()
            process_time_pre = time.time()
            driving_time_now = time.time()
            rospy.loginfo('dt: ' + str(driving_time_now - driving_time_pre))
            driving_time_pre = driving_time_now

            try:
                src = img_ori
                src = src[240:480, 0:640]
                re_frame = cv2.resize(src, (120, 64), interpolation=cv2.INTER_AREA)
                re_frame = [re_frame]
                result = self.predict(re_frame)
                rospy.loginfo(result)
    
                linear_vel = 0.18
                angular_vel = result[0][0] * 1.7
                self.fn_driving_deeplearning(linear_vel, angular_vel)
            except Exception as e:
                rospy.logerr("Fail : " + str(e))

            process_time_now = time.time()
            rospy.loginfo('process time: ' + str(process_time_now - process_time_pre))

    def cb_driving_mode(self, msg):
        self.driving_mode = msg.data

    def fn_stop(self):
        rospy.loginfo("[Process End] Shut down")
        rospy.sleep(0.3)
        twist_msg = Twist()

        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        self.pub_vel.publish(twist_msg)

    def _build_net(self):
        # 입력 받은 이름으로 변수 명을 설정한다.
        with tf.variable_scope(self.name):
            # Boolean Tensor 생성 for dropout
            # tf.layers.dropout( training= True/Fals) True/False에 따라서 학습인지 / 예측인지 선택하게 됨
            # default = False
            self.training = tf.placeholder(tf.bool)

            # 입력 그래프 생성
            self.X = tf.placeholder(tf.float32, [None, img_height, img_width, img_channel], name='X_im')
            self.Y = tf.placeholder(tf.float32, [None, 1])

            # Convolutional Layer1
            conv1 = tf.layers.conv2d(inputs=self.X, filters=32, kernel_size=[3, 3], padding='SAME',
                                     activation=tf.nn.relu)
            pool1 = tf.layers.max_pooling2d(inputs=conv1, pool_size=[2, 2], strides=2, padding="SAME")

            # Convolutional Layer2
            conv2 = tf.layers.conv2d(inputs=pool1, filters=64, kernel_size=[3, 3], padding='SAME',
                                     activation=tf.nn.relu)
            pool2 = tf.layers.max_pooling2d(inputs=conv2, pool_size=[2, 2], strides=2, padding='SAME')

            # Convolutional Layer3
            conv3 = tf.layers.conv2d(inputs=pool2, filters=128, kernel_size=[5, 5], padding='SAME',
                                     activation=tf.nn.relu)
            pool3 = tf.layers.max_pooling2d(inputs=conv3, pool_size=[2, 2], strides=2, padding='SAME')

            # Convolutional Layer4
            conv4 = tf.layers.conv2d(inputs=pool3, filters=128, kernel_size=[3, 3], padding='SAME',
                                     activation=tf.nn.relu)

            # Dropout Layer

            # Dense Layer4 with Relu
            flat = tf.reshape(conv4, [-1, 128 * 15 * 8])
            dropout1 = tf.layers.dropout(inputs=flat, rate=0.2, training=self.training)
            dense4 = tf.layers.dense(inputs=dropout1, units=128, activation=tf.nn.relu)
            dropout2 = tf.layers.dropout(inputs=dense4, rate=0.5, training=self.training)
            dense5 = tf.layers.dense(inputs=dropout2, units=128, activation=tf.nn.relu)
            dropout3 = tf.layers.dropout(inputs=dense5, rate=0.5, training=self.training)
            dense6 = tf.layers.dense(inputs=dropout3, units=64, activation=tf.nn.relu)
            self.logits = tf.layers.dense(inputs=dense6, units=1, name='logits')
            #self.softmax = tf.nn.softmax(self.logits, axis=None, name=None, dim=None)

        # Cost Function
        self.cost = tf.reduce_mean(tf.square(self.logits-self.Y))
        self.optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(self.cost)


    def train(self, x_data, y_data, training=False):
        return self.sess.run([self.cost, self.optimizer],
                             feed_dict={self.X: x_data, self.Y: y_data, self.training: training})

    def predict(self, x_test, training=False):
        return self.sess.run(self.logits, feed_dict={self.X: x_test, self.training: training})

    def restore(self, mode):
        # save_file = './dcgan_model/DCGAN_cnn.ckpt'

        if mode == 'test':
            save_file = '/home/nvidia/Auto-Mobile-Robot/deeplearning/driving_model/auto.ckpt'

            saver = tf.train.Saver()
            saver.restore(self.sess, save_file)

    @staticmethod
    def main():
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('Deeplearning_Driving_Node')
    print('\n' + 'Learning started...' + '\n')

    sess = tf.Session()
    m = DeeplearningDrivingNode(sess, "model")

    m.restore('test')
    m.main()

    # ret, frame = src.read()
    # re_frame =cv2.resize(frame,(120,64),interpolation=cv2.INTER_AREA)
    # cv2.imshow('d', re_frame)
    # re_frame = [re_frame]
    #res = m.predict(re_frame)
