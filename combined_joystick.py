#!/usr/bin/env python


import os
import cv2
import csv
import numpy as np
import sys
from math import cos, sin, atan, asin, pi, floor, ceil
import gui
import curses
import pdb

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, String
from sensor_msgs.msg import Joy
import time
import os
import ast

class LaserSubs(object):
    laser_ranges = 0

    def __init__(self):
        scan = LaserScan()
        self.init_laser_range()
        print("test", self.init_laser_range())
        rospy.Subscriber('/base_scan', LaserScan, self.LaserData)

    def LaserData(self,msg):
        self.laser_ranges = msg.ranges

    def init_laser_range(self):
        self.laser_ranges = None
        for i in range(3):
            # while self.laser_ranges is None:
            if self.laser_ranges is None:
                try:
                    laser_data = rospy.wait_for_message('/base_scan', LaserScan, timeout=5)
                    self.laser_ranges = laser_data.ranges
                    time.sleep(0.05)
                except:
                    print('Waiting for base_scan to be ready')
                    time.sleep(0.05)


class LidarProcessor(object):
    dist_slow = 0
    dist_stop = 0

    flag_f = 0
    flag_l = 0
    flag_r = 0
    flag_fl = 0
    flag_fr = 0
    flag_old = [flag_f,flag_l,flag_r,flag_fl,flag_fr]
    flag_new = [flag_f,flag_l,flag_r,flag_fl,flag_fr]

    dist_l = 0
    dist_fl = 0
    dist_f = 0
    dist_fr = 0
    dist_r = 0
    dist_cl = 0
    dist_cr = 0
    dist = [dist_l,dist_fl,dist_f,dist_fr,dist_r]

    pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=1)

    def __init__(self,settings):
        self.laser_subs_object = LaserSubs()
        print("test", self.laser_subs_object)
        self.init_distance(settings)

    def init_distance(self, settings):
        self.dist_stop = settings.get("platform_stop_dist")
        self.dist_slow  = settings.get("platform_clear_dist")
        # self.dist_slow = 200
        # self.dist_stop = 100
        print("dist stop:", self.dist_stop)
        print("dist slow:", self.dist_slow)

    def update_distance(self):
        # print (len(self.laser_subs_object.laser_ranges))
        # print (self.laser_subs_object)
        # Max array length [0:961]
        self.dist_r = self.calc_avg(self.laser_subs_object.laser_ranges[0:193])
        self.dist_fr = self.calc_avg(self.laser_subs_object.laser_ranges[193:386])
        self.dist_f = self.calc_avg(self.laser_subs_object.laser_ranges[386:579])
        self.dist_fl = self.calc_avg(self.laser_subs_object.laser_ranges[579:772])
        self.dist_l = self.calc_avg(self.laser_subs_object.laser_ranges[772:962])

        print ("right", self.dist_r)
        print ("left", self.dist_l)
        print ("front_right", self.dist_fr)
        print ("front_left", self.dist_fl)
        print ("front", self.dist_f)

        self.dist = [self.dist_l, self.dist_fl, self.dist_f, self.dist_fr, self.dist_r]

        self.save_data((self.dist_f))

        self.update_flags(self.dist_f,self.dist_l,self.dist_r,self.dist_fl,self.dist_fr)

    def update_speed(self):
        if self.flag_old != self.flag_new:
            if self.flag_f == 3 | self.flag_fl == 3 | self.flag_fr == 3:
                print("force stop")
                twist = Twist() # force stop if needed
                self.pub.publish(twist)
        pass

    def update_flags(self,dist_f,dist_l,dist_r,dist_fl,dist_fr):
        self.flag_old = [self.flag_f, self.flag_l, self.flag_r, self.flag_fl, self.flag_fr]
        self.flag_f = self.update_flag(dist_f)
        self.flag_l = self.update_flag(dist_l)
        self.flag_r = self.update_flag(dist_r)
        self.flag_fl = self.update_flag(dist_fl)
        self.flag_fr = self.update_flag(dist_fr)
        self.flag_new = [self.flag_f, self.flag_l, self.flag_r, self.flag_fl, self.flag_fr]

    def update_flag(self, dist):
        if dist >= self.dist_slow:
            flag = 1
        elif (dist >= self.dist_stop) & (dist <= self.dist_slow):
            flag = 2
        elif dist <= self.dist_stop:
            flag = 3
        else:
            flag = 3
        #flag = 1
        return flag

    @staticmethod
    def save_data(data):
        append_file = open("/home/sinapse/Desktop/MonkeyGUI-master/RewardData/LIDARData.txt", "a")
        # Save in a 2 by 2 array or it will print all integers vertically
        np.savetxt(append_file, [data], fmt="%f", delimiter=",")
        append_file.close()

    @staticmethod
    def calc_avg(values):
        return np.percentile(values,25)


class JoystickProcessor(object):
    #rosrun joy joy_node
    #rostopic echo joy
    
    speed_slow = 0
    speed_fast = 0

    pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=1)

    def __init__(self, settings, lidar):
        self.sub = rospy.Subscriber("joy", Joy, self.callback)
        self.lidar = lidar
        self.init_speed(settings)

    def init_speed(self,settings):
        self.speed_fast = settings.get("platform_normalSpeed")
        self.speed_slow = settings.get("platform_slowDownSpeed")
        # self.speed_slow = 0
        # self.speed_fast = 0
        print("speed slow:", self.speed_slow)
        print("speed fast:", self.speed_fast)

    def callback(self, data):
        print(["{:0.3f}".format(x) for x in self.lidar.dist])
        if(override == False):
            self.move(data)


    def move(self, data):
        twist = Twist()
        # print(data)

        # Data.axis[1] = joystick moving front and back, data.axis[0] = joystick moving left and right
        speed = data.axes[1] * 2
        angular_speed = data.axes[0]
        try:
            toggle_check = data.axes[3]
        except:
            toggle_check = 10 #use 10 as value to signify small joystick
        # angular_speed = angular_speed * -1 # for small joystick, inverted
        if abs(data.axes[1]) < abs(data.axes[0]):
            speed = 0
            angular_speed = data.axes[0]
        elif abs(data.axes[1]) >= abs(data.axes[0]):
            speed = data.axes[1] * 2
            angular_speed = 0

        if toggle_check == 10:
            angular_speed = angular_speed * -1 # for small joystick, inverted
        
        if toggle_check == -1.0: #Toggle to disable obstacle lock
            twist = self.move_forward(1, 1, 1, speed, twist)
        else: #whether toggle_check == 1.0 or 10, we apply the same constraints
            if speed > 0.2:  # Joystick is indicating to move forward
                twist =  self.move_forward(self.lidar.flag_f, self.lidar.flag_fl, self.lidar.flag_fr, speed, twist)
            if speed < 0:  # Joystick is indicating to move in a reverse directionmove_forwa
                #twist = self.move_forward(1, 1, 1, speed/2, twist)
                twist = self.move_forward(3, 3, 3, speed/2, twist) # Disable reverse

        # turn left twist.angular.z is positive, turn right twist.angular.z is negative
        # turn left data.axes[0] is positive, turn right, data.axes[0] is negative
        if toggle_check == -1.0: #Toggle to disable obstacle lock
            twist = self.move_sideway(angular_speed, 1, 1, twist)
        else:
            if angular_speed > 0.2:
                twist = self.move_sideway(angular_speed, self.lidar.flag_fl, self.lidar.flag_l, twist)
            if angular_speed < -0.2:
                twist = self.move_sideway(angular_speed, self.lidar.flag_fr, self.lidar.flag_r, twist)
        global clamp
        if clamp == False:
            self.pub.publish(twist)

    def move_forward(self, flag_front, flag_frontleft, flag_frontright, multiplier, twist):        
        flag_frontside = max(flag_frontleft, flag_frontright)
	
        slow_scale_factor = (self.lidar.dist_f - self.lidar.dist_stop) / (self.lidar.dist_slow - self.lidar.dist_stop)
        slow_scale = (6 * slow_scale_factor) / (1 + (6 * slow_scale_factor))
	#pdb.set_trace()
        #flag_frontside = 1

        if (flag_front == 3) | (flag_frontside == 3):
            twist.linear.x = 0
            print("flag 3")
        elif (flag_front == 2):
            twist.linear.x = self.speed_fast * multiplier * slow_scale
            #twist.linear.x = self.speed_slow * multiplier
            print("flag 2")
        elif (flag_front == 1):
            twist.linear.x = self.speed_fast * multiplier
            print("flag 1")

        print("x: ", twist.linear.x)
        print("x: ", multiplier)
        return twist

    def move_sideway(self, angular_speed, flag_frontside, flag_side, twist):
        
        if (flag_side == 3) | (flag_frontside == 3):
            twist.angular.z = 0
            print("flag 3")
	    #print("frontside", flag_frontside)
	    #print("side", flag_side)
            #twist.angular.z = self.speed_slow * angular_speed * -2.5
        elif ((flag_frontside <= 2) & (flag_side >= 2) | (flag_frontside >= 2) & (flag_side <= 2)):
            twist.angular.z = self.speed_fast * angular_speed * 2.5
            print("flag 2")
        elif (flag_frontside <= 2) & (flag_side == 1):
            twist.angular.z = self.speed_fast * angular_speed * 2.5
            print("flag 1")

        print("z: ", twist.angular.z)
        print("z: ", angular_speed)
        return twist


def get_gui(data = None):
    if data is None:
        settings = {
            "platform_stop_dist": 0.6,
            "platform_clear_dist": 1.5,
            "platform_normalSpeed": 0.2,
            "platform_slowDownSpeed": 0.1
        }
        return settings
    global lidar
    global joystick
    settings = ast.literal_eval(data.data)
    try:
        lidar.init_distance(settings)
        joystick.init_speed(settings)
    except Exception as e:
        print('Error:', e)

def controllerCallback(data):
    global clamp
    print('triggered')
    if data.data//10 == 2:
        clamp = False
    elif data.data//10 == 3 or data.data//10 == 4:
        clamp = True
    print ("clamp", clamp)

def overrideSettings(data):
    global override
    if data.data == 2:
        override = True
    elif data.data == 1:
        override = False

if __name__ == '__main__':
    rospy.init_node('controller_joystick')

    clamp = False
    emergency = False
    override = False
    settings = get_gui()
    global lidar
    global joystick
    lidar = LidarProcessor(settings)
    joystick = JoystickProcessor(settings,lidar)

    rospy.Subscriber('trigger_msgs', Int16, controllerCallback)
    rospy.Subscriber('gui_settings', String, get_gui)
    rospy.Subscriber('override_msgs', Int16, overrideSettings)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        lidar.update_distance()
        lidar.update_speed()

        rate.sleep()
