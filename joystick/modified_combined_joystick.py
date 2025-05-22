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
    rear_laser_ranges = 0

    def __init__(self):
        scan = LaserScan()
        self.init_laser_range()
        print("test", self.init_laser_range())
        rospy.Subscriber('/base_scan', LaserScan, self.LaserData)
        rospy.Subscriber('/rear_scan', LaserScan, self.RearLaserData)  # New rear lidar subscriber

    def LaserData(self, msg):
        self.laser_ranges = msg.ranges

    def RearLaserData(self, msg):
        self.rear_laser_ranges = msg.ranges

    def init_laser_range(self):
        self.laser_ranges = None
        self.rear_laser_ranges = None
        
        # Initialize front lidar
        for i in range(3):
            if self.laser_ranges is None:
                try:
                    laser_data = rospy.wait_for_message('/base_scan', LaserScan, timeout=5)
                    self.laser_ranges = laser_data.ranges
                    time.sleep(0.05)
                except:
                    print('Waiting for base_scan to be ready')
                    time.sleep(0.05)
        
        # Initialize rear lidar
        for i in range(3):
            if self.rear_laser_ranges is None:
                try:
                    rear_laser_data = rospy.wait_for_message('/rear_scan', LaserScan, timeout=5)
                    self.rear_laser_ranges = rear_laser_data.ranges
                    time.sleep(0.05)
                except:
                    print('Waiting for rear_scan to be ready')
                    time.sleep(0.05)


class LidarProcessor(object):
    dist_slow = 0
    dist_stop = 0

    # Front lidar flags
    flag_f = 0
    flag_l = 0
    flag_r = 0
    flag_fl = 0
    flag_fr = 0
    
    # Rear lidar flags
    flag_rl = 0  # rear left
    flag_rr = 0  # rear right
    flag_rb = 0  # rear back (center)
    
    flag_old = [flag_f, flag_l, flag_r, flag_fl, flag_fr, flag_rl, flag_rr, flag_rb]
    flag_new = [flag_f, flag_l, flag_r, flag_fl, flag_fr, flag_rl, flag_rr, flag_rb]

    # Front distances
    dist_l = 0
    dist_fl = 0
    dist_f = 0
    dist_fr = 0
    dist_r = 0
    dist_cl = 0
    dist_cr = 0
    
    # Rear distances
    dist_rl = 0  # rear left
    dist_rr = 0  # rear right
    dist_rb = 0  # rear back
    
    dist = [dist_l, dist_fl, dist_f, dist_fr, dist_r]
    rear_dist = [dist_rl, dist_rb, dist_rr]

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def __init__(self, settings):
        self.laser_subs_object = LaserSubs()
        print("test", self.laser_subs_object)
        self.init_distance(settings)

    def init_distance(self, settings):
        self.dist_stop = settings.get("platform_stop_dist")
        self.dist_slow = settings.get("platform_clear_dist")
        print("dist stop:", self.dist_stop)
        print("dist slow:", self.dist_slow)        

    def update_distance(self):
        # Front lidar processing (existing code)
        if self.laser_subs_object.laser_ranges is not None:
            self.dist_r = self.calc_avg(self.laser_subs_object.laser_ranges[0:193])
            self.dist_fr = self.calc_avg(self.laser_subs_object.laser_ranges[193:386])
            self.dist_f = self.calc_avg(self.laser_subs_object.laser_ranges[386:579])
            self.dist_fl = self.calc_avg(self.laser_subs_object.laser_ranges[579:772])
            self.dist_l = self.calc_avg(self.laser_subs_object.laser_ranges[772:962])

        # Rear lidar processing (new)
        if self.laser_subs_object.rear_laser_ranges is not None:
            # Adjust these ranges based on your rear lidar orientation and shelf clearance
            # You may need to mask out the shelf area
            rear_ranges = self.filter_shelf_points(self.laser_subs_object.rear_laser_ranges)
            
            # Assuming rear lidar has similar 270-degree coverage
            # Adjust sectors based on your specific mounting and requirements
            self.dist_rl = self.calc_avg(rear_ranges[0:320])     # rear left
            self.dist_rb = self.calc_avg(rear_ranges[320:640])   # rear back
            self.dist_rr = self.calc_avg(rear_ranges[640:961])   # rear right

        self.dist = [self.dist_l, self.dist_fl, self.dist_f, self.dist_fr, self.dist_r]
        self.rear_dist = [self.dist_rl, self.dist_rb, self.dist_rr]

        self.save_data((self.dist_f))
        self.update_flags(self.dist_f, self.dist_l, self.dist_r, self.dist_fl, self.dist_fr,
                         self.dist_rl, self.dist_rr, self.dist_rb)

    def filter_shelf_points(self, rear_ranges):
        """
        Filter out points that correspond to the shelf structure
        Adjust these parameters based on your shelf geometry and lidar mounting
        """
        filtered_ranges = list(rear_ranges)
        
        # Example: If shelf blocks certain angles, set those to max range
        # You'll need to determine these angles based on your setup
        shelf_start_angle = 100  # example indices where shelf blocks view
        shelf_end_angle = 200
        
        for i in range(shelf_start_angle, min(shelf_end_angle, len(filtered_ranges))):
            filtered_ranges[i] = 10.0  # Set to max safe distance or filter out
            
        return filtered_ranges

    def update_speed(self):
        if self.flag_old != self.flag_new:
            if (self.flag_f == 3 | self.flag_fl == 3 | self.flag_fr == 3 | 
                self.flag_rl == 3 | self.flag_rr == 3 | self.flag_rb == 3):
                print("force stop")
                twist = Twist()  # force stop if needed
                self.pub.publish(twist)

    def update_flags(self, dist_f, dist_l, dist_r, dist_fl, dist_fr, dist_rl, dist_rr, dist_rb):
        self.flag_old = [self.flag_f, self.flag_l, self.flag_r, self.flag_fl, self.flag_fr,
                        self.flag_rl, self.flag_rr, self.flag_rb]
        
        # Front flags (existing)
        self.flag_f = self.update_flag(dist_f)
        self.flag_l = self.update_flag(dist_l)
        self.flag_r = self.update_flag(dist_r)
        self.flag_fl = self.update_flag(dist_fl)
        self.flag_fr = self.update_flag(dist_fr)
        
        # Rear flags (new)
        self.flag_rl = self.update_flag(dist_rl)
        self.flag_rr = self.update_flag(dist_rr)
        self.flag_rb = self.update_flag(dist_rb)
        
        self.flag_new = [self.flag_f, self.flag_l, self.flag_r, self.flag_fl, self.flag_fr,
                        self.flag_rl, self.flag_rr, self.flag_rb]

    def update_flag(self, dist):
        if dist >= self.dist_slow:
            flag = 1
        elif (dist >= self.dist_stop) & (dist <= self.dist_slow):
            flag = 2
        elif dist <= self.dist_stop:
            flag = 3
        else:
            flag = 3
        return flag

    @staticmethod
    def save_data(data):
        append_file = open("/home/sinapse/Desktop/MonkeyGUI-master/RewardData/LIDARData.txt", "a")
        np.savetxt(append_file, [data], fmt="%f", delimiter=",")
        append_file.close()

    @staticmethod
    def calc_avg(values):
        return np.percentile(values, 50)


class JoystickProcessor(object):
    speed_slow = 0
    speed_fast = 0
    move_front = 1
    move_left = 1
    move_right = 0

    pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=1)

    def __init__(self, settings, lidar):
        self.sub = rospy.Subscriber("joy", Joy, self.callback)
        self.lidar = lidar
        self.init_speed(settings)

    def init_speed(self, settings):
        self.speed_fast = settings.get("platform_normalSpeed")
        self.speed_slow = settings.get("platform_slowDownSpeed")
        self.move_front = settings.get("platform_move_front")
        self.move_left = settings.get("platform_move_left")
        self.move_right = settings.get("platform_move_right")

    def callback(self, data):
        print("Front: " + str(["{:0.3f}".format(x) for x in self.lidar.dist]))
        print("Rear:  " + str(["{:0.3f}".format(x) for x in self.lidar.rear_dist]))
        if(override == False):
            self.move(data)

    def move(self, data):
        twist = Twist()
        speed = data.axes[1] * 2
        angular_speed = data.axes[0]
        
        try:
            toggle_check = data.axes[3]
        except:
            toggle_check = 10

        if toggle_check == 10:
            angular_speed = angular_speed * -1

        # Disable movements based on settings
        if self.move_front == 0:
            speed = 0
        if (self.move_left == 0) & (angular_speed > 0):
            angular_speed = 0
        if (self.move_right == 0) & (angular_speed < 0):
            angular_speed = 0
        
        if toggle_check == -1.0:  # Toggle to disable obstacle lock
            twist = self.move_forward(1, 1, 1, speed, twist)
        else:
            if speed > 0.6:  # Forward movement
                twist = self.move_forward(self.lidar.flag_f, self.lidar.flag_fl, self.lidar.flag_fr, speed, twist)
            if speed < 0:  # Reverse movement
                twist = self.move_forward(3, 3, 3, speed/2, twist)  # Disable reverse

        # Enhanced turning with rear obstacle avoidance
        if toggle_check == -1.0:  # Toggle to disable obstacle lock
            twist = self.move_sideway(angular_speed, 1, 1, 1, 1, twist)
        else:
            if (angular_speed > 0.8) & (speed > -1):  # Turn left
                twist = self.move_sideway(angular_speed, self.lidar.flag_fl, self.lidar.flag_l, 
                                        self.lidar.flag_rl, self.lidar.flag_rb, twist)
            if (angular_speed < -0.8) & (speed > -1):  # Turn right
                twist = self.move_sideway(angular_speed, self.lidar.flag_fr, self.lidar.flag_r,
                                        self.lidar.flag_rr, self.lidar.flag_rb, twist)
        
        global clamp
        print("global clamp value", clamp)
        if clamp == False:
            print(twist)
            self.pub.publish(twist)

    def move_forward(self, flag_front, flag_frontleft, flag_frontright, multiplier, twist):        
        flag_frontside = max(flag_frontleft, flag_frontright)
        slow_scale_factor = (self.lidar.dist_f - self.lidar.dist_stop) / (self.lidar.dist_slow - self.lidar.dist_stop)
        slow_scale = (6 * slow_scale_factor) / (1 + (6 * slow_scale_factor))

        if (flag_front == 3) | (flag_frontside == 3):
            twist.linear.x = 0
            print("flag 3")
        elif (flag_front == 2):
            twist.linear.x = self.speed_fast * multiplier * slow_scale
            print("flag 2")
        elif (flag_front == 1):
            twist.linear.x = self.speed_fast * multiplier
            print("flag 1")

        return twist

    def move_sideway(self, angular_speed, flag_frontside, flag_side, flag_rear_side, flag_rear_back, twist):
        """
        Enhanced turning with rear obstacle consideration
        
        Args:
            angular_speed: Joystick angular input
            flag_frontside: Front side obstacle flag (fl for left turn, fr for right turn)
            flag_side: Side obstacle flag (l for left turn, r for right turn)  
            flag_rear_side: Rear side obstacle flag (rl for left turn, rr for right turn)
            flag_rear_back: Rear back obstacle flag
        """
        
        # Check if any critical obstacle is detected
        critical_obstacle = (flag_side == 3) | (flag_frontside == 3) | (flag_rear_side == 3) | (flag_rear_back == 3)
        
        # Check if any obstacle is in slow zone
        slow_zone_obstacle = (flag_side == 2) | (flag_frontside == 2) | (flag_rear_side == 2) | (flag_rear_back == 2)
        
        if critical_obstacle:
            twist.angular.z = 0
            print("Turning blocked - critical obstacle detected")
            print(f"Side: {flag_side}, Front-side: {flag_frontside}, Rear-side: {flag_rear_side}, Rear-back: {flag_rear_back}")
        elif slow_zone_obstacle:
            # Reduce turning speed when obstacles are in slow zone
            twist.angular.z = self.speed_slow * angular_speed * 2.5
            print("Turning slowed - obstacle in slow zone")
        else:
            # Normal turning speed
            twist.angular.z = self.speed_fast * angular_speed * 2.5
            print("Normal turning speed")

        print("Angular speed input:", angular_speed)
        return twist


# Rest of the functions remain the same
def get_gui(data=None):
    if data is None:
        settings = {
            "platform_stop_dist": 0.6,
            "platform_clear_dist": 1.5,
            "platform_normalSpeed": 0.2,
            "platform_slowDownSpeed": 0.1,
            "platform_move_front": 1,
            "platform_move_left": 1,
            "platform_move_right": 1
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
    print("clamp", clamp)

def overrideSettings(data):
    global override
    if data.data == 2:
        override = True
    elif data.data == 1:
        override = False

if __name__ == '__main__':
    rospy.init_node('controller_joystick')

    clamp = True
    emergency = False
    override = False
    settings = get_gui()
    global lidar
    global joystick
    lidar = LidarProcessor(settings)
    joystick = JoystickProcessor(settings, lidar)

    rospy.Subscriber('trigger_msgs', Int16, controllerCallback)
    rospy.Subscriber('gui_settings', String, get_gui)
    rospy.Subscriber('override_msgs', Int16, overrideSettings)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        lidar.update_distance()
        lidar.update_speed()
        rate.sleep()
