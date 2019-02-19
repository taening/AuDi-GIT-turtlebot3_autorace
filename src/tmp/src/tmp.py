#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import tf
from sensor_msgs.msg import Imu


class Compass:
    def __init__(self):
	self.pi = math.pi
	self.sub_imu = rospy.Subscriber('/imu', Imu, self.cb_orientation_receive)	

    def cb_orientation_receive(self, msg):
	quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	
	degree = int(180. * yaw / self.pi) 
	if degree < 0:
	    degree += 360.

	print("roll: " + str(roll))
	print("pitch: " + str(pitch))
	print("yaw: " + str(yaw))
	print("degree: " + str(degree))
	print("========================")

    @staticmethod
    def main():
	rospy.spin()


if __name__ == '__main__':
    rospy.init_node("Compass_Node")
    node = Compass()
    node.main()
