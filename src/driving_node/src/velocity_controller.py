#!/usr/bin/env python
# -*- coding: utf-8 -*-
from geometry_msgs.msg import Twist
import rospy
import time
import math
import copy


class VelocityController:
    def __init__(self):
        self.t_pre = time.time()
        self.dt = 0.1

        self._last_err = 0.0
        self._sum_err = 0.0

        self.twist = Twist()

        #self.right_obstacle = []
        #self.forward_obstacle1 = []
        #self.forward_obstacle2 = []

        self.count = 0
        self.last_D_err = 0
        self.last_angular = 0
        self.sum_I = 0
        self.odom_theta_start = 0.0
        self.odom_theta_last = 0
        self.delta = 0
        self.sum = 0

        self.last_err_d = 0.0

    def fn_pid_control(self, linear_vel, err, kp, ki, kd):
        twist = Twist()

        ang_p = kp * err
        ang_i = ki * self._sum_err
        ang_d = kd * (err - self._last_err) / self.dt

        self._sum_err += err * self.dt
        self._last_err = err

        twist.linear.x = linear_vel
        twist.angular.z = -float(ang_p + ang_i + ang_d)
        return twist

    def fn_vel_control(self, linear_vel, angular_vel):
        twist = Twist()

        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        return twist

    def _fn_tunnel_odom_angle(self, theta1, theta2):
        theta = theta2 - theta1 + self.sum
        if theta - self.odom_theta_last < -6:
            self.delta = 2*math.pi
        elif theta - self.odom_theta_last > 6:
            self.delta = -2*math.pi
        else :
            self.delta = 0
        self.sum += self.delta
        theta += self.delta
        self.odom_theta_last = theta
        rospy.logwarn('{:.6f}, {:.6f}, {:.6f}'.format(theta1, theta2, theta))
        return theta

    def fn_tunnel_init(self, odom_theta_current):
        self.count = 0
        self.last_D_err = 0
        self.last_angular = 0
        self.sum_I = 0
        self.odom_theta_start = odom_theta_current

    def fn_tunnel_control(self, list_scan_dis, odom_theta_current):
        for i in range(-180, 40):
            if list_scan_dis[i] > 0.5 or list_scan_dis[i] < 0.01:
                list_scan_dis[i] = 0.5

        odom_theta = -self._fn_tunnel_odom_angle(odom_theta_current, self.odom_theta_start)

        check_right = min(list_scan_dis[-60:-20])  # -90:-1
        forward_left_min = min(list_scan_dis[-5:] + list_scan_dis[:20])
        check_right2 = min(list_scan_dis[-90:-30])
        forward_sort = copy.copy(list_scan_dis[-35:] + list_scan_dis[:20])
        forward_sort.sort()
        forward_min = sum(forward_sort[:3]) / 3
        #forward_right_min = min(list_scan_dis[-32:-1])

        #forward_left_min = min(list_scan_dis[0:20])

        #forward_min = min([forward_left_min, forward_right_min])

        right_sort = copy.copy(list_scan_dis[-120:-60])

        right_sort.sort()
        min_right = min(list_scan_dis[-100:-80])

        right_average = sum(right_sort[:10]) / 10

        right_min = min(list_scan_dis[-120:-60])

        P_err = right_average - 0.15  # 0.17

        right_forward_sort = copy.copy(list_scan_dis[-90:-60])

        right_forward_sort.sort()

        right_forward_average = sum(right_forward_sort[:5]) / 5

        right_backward_sort = copy.copy(list_scan_dis[-120:-90])

        right_backward_sort.sort()

        right_backward_average = sum(right_backward_sort[:5]) / 5

        D_err = right_forward_average - 0.15  # 0.17

        #length = list(msg.intesities)
        #for i in range(-180, 40):
        #    if length[i] < 0.01:
        #        length[i] = 4

        #check_right = min(length[-90:-1])
        #forward_right_min = min(length[-40:-1])
        #forward_left_min = min(length[0:40])
        #forward_min = min([forward_left_min, forward_right_min])
        #right_average = sum(length[-120:-60])/60
        #P_err = right_average - 0.18
        #right_forward = msg.ranges[-90:-60]
        #right_forward_min = min(right_forward)
        #right_forward_average = sum(length[-90:-60])/30
        #D_err = right_forward_average - 0.18

        #if forward_min < 0.25 and forward_right_average < 0.35:

        if forward_min < 0.23:
            if check_right > 0.24 and odom_theta > 0.2:
                self.twist.angular.z = -1.6
                #self.twist.linear.x = 0.1 * float((1 - 0.2 / forward_min) ** 2)
                self.twist.linear.x = 0
                self.count=1

            else:
                if forward_left_min > 0.23:
                    self.twist.angular.z = 0.7
                    self.twist.linear.x = 0

                else:
                    self.twist.angular.z = 1.8
                    self.twist.linear.x = 0
                    #self.twist.linear.x = 0.1 * float((1 - 0.2 / forward_min) ** 2)
                    self.count=2

        else:
            if odom_theta <= 0.2:
                self.twist.angular.z = 0
                self.twist.linear.x = 0.2
                self.count = 7
            else:
                if right_forward_average > 0.19 and check_right2 > 0.16:

        
                    kp = 15 #18
                    kd = 5 # 7
                    P_err = right_forward_average - 0.145 # 0.15
                    self.count=4

                else:
                    if right_backward_average>right_forward_average :
                        kp = -7
                        kd = 0
                        self.count = 3

                    elif right_backward_average<right_forward_average and right_backward_average>0.08 :

                        kp = 3

                        P_err = right_forward_average - right_backward_average #0.15

                        kd = 0
                    elif right_min > 0.13:

                        kp = 10 # 10
                        P_err = right_forward_average - 0.155 #0.15
                        kd = 0
                        D_err = right_forward_average - 0.155
                        self.count = 5

                    else:

                        kp = 10

                        kd = 0
                        P_err = right_min - 0.125
                        self.count = 6
                self.sum_I = self.sum_I + P_err + D_err
            #self.sum_I = self.sum_I + right_average -0.17

                ki = 0 # 3
                I_control = ki * self.sum_I
                P_control = kp * P_err
                D_control = kd * (D_err - self.last_D_err)
                self.last_D_err = D_err
                self.twist.angular.z = -float(P_control + D_control)
                self.twist.linear.x = 0.2 * (3**(-abs(D_err)*2))
            #self.twist.linear.x = 0.15*(-69.4*(D_err)**2+1)
            #self.twist.linear.x = 0.2 * (3**(-abs(D_err)*2))
        #if self.twist.angular.z - self.last_angular > 0.8 or self.last_angular - self.twist.angular.z > 0.8:
            #self.twist.angular.z = (self.last_angular+self.twist.angular.z)/2
        #else :
            #self.twist.angular.z = self.twist.angular.z

        #print(right_forward_average, right_backward_average, right_min, self.count, self.twist.angular.z, self.theta)
        self.last_angular = self.twist.angular.z

        return self.twist



    def fn_frog_control(self, list_scan_dis):
        for i in range(-180, 40):
            if list_scan_dis[i] > 0.5 or list_scan_dis[i] < 0.01:
                list_scan_dis[i] = 0.5


        check_right = min(list_scan_dis[-70:-20]) 
        check_right2 = min(list_scan_dis[-90:-30])
        forward_right_min = min(list_scan_dis[-34:-1])#-34
        forward_left_min = min(list_scan_dis[0:20]) #20
        forward_min = min([forward_left_min, forward_right_min])
        right_sort = copy.copy(list_scan_dis[-120:-60])
        right_sort.sort()
        #print(right_sort)
        right_average = sum(right_sort[:10])/10
        #print(right_average)
        right_min = min(list_scan_dis[-120:-60])
        err_p = right_average - 0.15 #0.15
        right_forward_sort = copy.copy(list_scan_dis[-90:-60])
        right_forward_sort.sort()
        right_forward_average = sum(right_forward_sort[:5])/5
        err_d = right_forward_average - 0.15 #0.15

        if forward_min < 0.22 : #0.22
            if check_right > 0.29:
                angular = -1
                linear = 0.1 * float((1 - 0.15 / forward_min) ** 2)

            else :
                angular = 0.8
                linear = 0.1 * float((1 - 0.15 / forward_min) ** 2)

            twist_msg = Twist()
            twist_msg.linear.x = linear
            twist_msg.angular.z = angular

        else :
            if right_forward_average > 0.19 and check_right2 > 0.16:
                kp = 18 #8
                kd = 7 #7
                err_p = right_forward_average - 0.15
            else:
                if right_min > 0.14:
                    kp = 10 #5
                    kd = 15 #10
                else :
                    kp = 2 #5
                    kd = 0 #5
                    err_p = right_min - 0.15
            twist_msg = Twist()

            ang_p = kp * err_p
            ang_d = kd * (err_d - self.last_err_d)

            self.last_err_d = err_d

            twist_msg.angular.z = -float(ang_p + ang_d)
            twist_msg.linear.x = 0.15 * (3**(-abs(err_d)*2))
            if twist_msg.angular.z - self.last_angular > 0.8 or self.last_angular - twist_msg.angular.z > 0.8:
                twist_msg.angular.z = (self.last_angular + twist_msg.angular.z)/2
            else :
                twist_msg.angular.z = twist_msg.angular.z
            self.last_angular = twist_msg.angular.z

        return twist_msg


        #twist_msg = self.controller.fn_frog_control(list_scan_dis)

    def fn_kh_control(self, list_scan_dis):
        for i in range(-180, 40):
            if list_scan_dis[i] > 0.5 or list_scan_dis[i] < 0.01:
                list_scan_dis[i] = 0.5


        check_right = min(list_scan_dis[-60:-20])  # -90:-1
        forward_left_min = min(list_scan_dis[-10:] + list_scan_dis[:20])
        check_right2 = min(list_scan_dis[-90:-30])
        forward_sort = copy.copy(list_scan_dis[-35:] + list_scan_dis[:20])
        forward_sort.sort()
        forward_min = sum(forward_sort[:3]) / 3
        rospy.logwarn(forward_min)
        rospy.logwarn(forward_sort)
        #forward_right_min = min(list_scan_dis[-32:-1])

        #forward_left_min = min(list_scan_dis[0:20])

        #forward_min = min([forward_left_min, forward_right_min])

        right_sort = copy.copy(list_scan_dis[-120:-60])

        right_sort.sort()
        min_right = min(list_scan_dis[-100:-80])

        right_average = sum(right_sort[:10]) / 10

        right_min = min(list_scan_dis[-120:-60])

        P_err = right_average - 0.15  # 0.17

        right_forward_sort = copy.copy(list_scan_dis[-90:-60])

        right_forward_sort.sort()

        right_forward_average = sum(right_forward_sort[:5]) / 5

        right_backward_sort = copy.copy(list_scan_dis[-120:-90])

        right_backward_sort.sort()

        right_backward_average = sum(right_backward_sort[:5]) / 5

        D_err = right_forward_average - 0.15  # 0.17

        #length = list(msg.intesities)
        #for i in range(-180, 40):
        #    if length[i] < 0.01:
        #        length[i] = 4

        #check_right = min(length[-90:-1])
        #forward_right_min = min(length[-40:-1])
        #forward_left_min = min(length[0:40])
        #forward_min = min([forward_left_min, forward_right_min])
        #right_average = sum(length[-120:-60])/60
        #P_err = right_average - 0.18
        #right_forward = msg.ranges[-90:-60]
        #right_forward_min = min(right_forward)
        #right_forward_average = sum(length[-90:-60])/30
        #D_err = right_forward_average - 0.18

        #if forward_min < 0.25 and forward_right_average < 0.35:

        if forward_min < 0.23:
            if check_right > 0.24:
                self.twist.angular.z = -1.6
                #self.twist.linear.x = 0.1 * float((1 - 0.2 / forward_min) ** 2)
                self.twist.linear.x = 0
                self.count=1

            else:
                if forward_left_min > 0.23:
                    self.twist.angular.z = 0.7
                    self.twist.linear.x = 0

                else:
                    self.twist.angular.z = 1.8
                    self.twist.linear.x = 0
                    #self.twist.linear.x = 0.1 * float((1 - 0.2 / forward_min) ** 2)
                    self.count=2


        else:
            if right_forward_average > 0.19 and check_right2 > 0.16:

        
                kp = 15 #18
                kd = 5 # 7
                P_err = right_forward_average - 0.145 # 0.15
                self.count=4

            else:
                if right_backward_average>right_forward_average :
                    kp = -7
                    kd = 0
                    self.count = 3

                elif right_backward_average<right_forward_average and right_backward_average>0.08 :

                    kp = 3

                    P_err = right_forward_average - right_backward_average #0.15

                    kd = 0
                elif right_min > 0.13:

                    kp = 10 # 10
                    P_err = right_forward_average - 0.155 #0.15
                    kd = 0
                    D_err = right_forward_average - 0.155
                    self.count = 5

                else:

                    kp = 10

                    kd = 0
                    P_err = right_min - 0.125
                    self.count = 6
            self.sum_I = self.sum_I + P_err + D_err
            #self.sum_I = self.sum_I + right_average -0.17

            ki = 0 # 3
            I_control = ki * self.sum_I
            P_control = kp * P_err
            D_control = kd * (D_err - self.last_D_err)
            self.last_D_err = D_err
            self.twist.angular.z = -float(P_control + D_control)
            self.twist.linear.x = 0.2 * (3**(-abs(D_err)*2))
            #self.twist.linear.x = 0.15*(-69.4*(D_err)**2+1)
            #self.twist.linear.x = 0.2 * (3**(-abs(D_err)*2))
        #if self.twist.angular.z - self.last_angular > 0.8 or self.last_angular - self.twist.angular.z > 0.8:
            #self.twist.angular.z = (self.last_angular+self.twist.angular.z)/2
        #else :
            #self.twist.angular.z = self.twist.angular.z

        #print(right_forward_average, right_backward_average, right_min, self.count, self.twist.angular.z, self.theta)
        self.last_angular = self.twist.angular.z

        return self.twist


if __name__ == '__main__':
    controller = VelocityController()
