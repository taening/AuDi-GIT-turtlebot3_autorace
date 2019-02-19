#!/usr/bin/env python

import rospy
import os,sys
from std_msgs.msg import UInt8


class TunnelNavigationNode:
    def __init__(self):
        self.sub_signal_navi = rospy.Subscriber('/mission/signal/navi', UInt8, self.cb_signal_navi, queue_size=1)

    def cb_signal_navi(self, msg):
        navigation_signal = msg.data
        if navigation_signal == 1:
            os.system('roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml')

    @staticmethod
    def main():
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Tunnel_Navigation_Node')
    node = TunnelNavigationNode()
    node.main()
