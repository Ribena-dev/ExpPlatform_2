#!/usr/bin/env python
#simplified version of the combined joystick that sub to /lidar_distance 
#lidar_process.py 
# broke turning left, right and going straight into seperate fucntions


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, String, Float32MultiArray
from sensor_msgs.msg import Joy
import time
import ast
import numpy as np

class ObstacleFlags:
   
    def __init__(self, dist_stop=0.6, dist_slow=1.5):
        self.dist_stop = dist_stop
        self.dist_slow = dist_slow
        
        # Front lidar distances: [left, front-left, front, front-right, right]
        self.front_left = 0.0
        self.front_front_left = 0.0
        self.front_front = 0.0
        self.front_front_right = 0.0
        self.front_right = 0.0
        
        # Rear lidar distances: [rear-left, back-left, back, back-right, rear-right]
        self.rear_left = 0.0
        self.rear_back_left = 0.0
        self.rear_back = 0.0
        self.rear_back_right = 0.0
        self.rear_right = 0.0
        
        
        # Flags (1=clear, 2=slow, 3=stop)
        self.flag_front = 1
        self.flag_left = 1
        self.flag_right = 1
        self.flag_back = 3
        self.flag_front_left = 1
        self.flag_front_right = 1
        self.flag_back_left = 1
        self.flag_back_right = 1
        
    def update_distances(self, combined_distances):
        # Update distances from lidar processor
        # Expected format: [front_left, front_front_left, front_front, front_front_right, front_right,
        #                  rear_left, rear_back_left, rear_back, rear_back_right, rear_right]
        
        if len(combined_distances) >= 10:
            # Front lidar distances
            self.front_left = combined_distances[0]
            self.front_front_left = combined_distances[1]
            self.front_front = combined_distances[2]
            self.front_front_right = combined_distances[3]
            self.front_right = combined_distances[4]
            
            # Rear lidar distances
            self.rear_left = combined_distances[5]
            self.rear_back_left = combined_distances[6]
            self.rear_back = combined_distances[7]
            self.rear_back_right = combined_distances[8]
            self.rear_right = combined_distances[9]
            
         
	    #print(combined_distances) 
            # Update all flags
            self.update_flags()
        else:

           print("Expected 10 distance values, got", {len(combined_distances)})

           print("failed to update distances")

        
    def update_flags(self):
        
        self.flag_front = self.calc_flag(self.front_front)
        self.flag_left = self.calc_flag(self.front_left)
        self.flag_right = self.calc_flag(self.front_right)
        self.flag_front_left = self.calc_flag(self.front_front_left)
        self.flag_front_right = self.calc_flag(self.front_front_right)
        self.flag_back_left = self.calc_flag(self.rear_back_left)
        self.flag_back_right = self.calc_flag(self.rear_back_right)
        self.flags=[self.flag_front,self.flag_left,self.flag_right,self.flag_front_left,self.flag_front_right,self.flag_back_left,self.flag_back_right]
<<<<<<< HEAD
        print(self.flags)  
=======
        print(self.flags)
>>>>>>> 9ee311e1373b0ad968b2a48091bc1fe4c21414ad

    def calc_flag(self, distance):
    
        if distance >= self.dist_slow :
            return 1  # Clear
        elif distance >= self.dist_stop:
            return 2  # Slow down
        else:
            return 3  # Stop
            
    def update_thresholds(self, dist_stop, dist_slow):
        #Update distance thresholds
        self.dist_stop = dist_stop
        self.dist_slow = dist_slow
        self.update_flags()


class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_controller', anonymous=True)
        
        # Movement settings
        self.speed_fast = 0.2
        self.speed_slow = 0.1
        self.move_front = 1
        self.move_left = 1
        self.move_right = 1
        
        # Global control flags
        self.clamp = False
        self.override = False
        
        # Obstacle detection
        self.obstacles = ObstacleFlags()
        
        # Publishers
        self.cmd_pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=1)
        self.emergency_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joystick_callback)
        self.distance_sub = rospy.Subscriber('/lidar_distances', Float32MultiArray, self.distance_callback)
        
        # Control subscribers
        self.trigger_sub = rospy.Subscriber('trigger_msgs', Int16, self.trigger_callback)
        
        self.override_sub = rospy.Subscriber('override_msgs', Int16, self.override_callback)
        
        # Initialize with default settings
        self.init_settings()
        
        rospy.loginfo("Joystick Controller initialized")
        rospy.loginfo("Waiting for lidar distance data...")
        
    def init_settings(self):
        
        settings = {
            "platform_stop_dist": 0.6,
            "platform_clear_dist": 1.5,
            "platform_normalSpeed": 0.2,
            "platform_slowDownSpeed": 0.1,
            "platform_move_front": 1,
            "platform_move_left": 1,
            "platform_move_right": 1
        }
        self.update_settings(settings)
        
    def update_settings(self, settings):
        
        self.speed_fast = settings.get("platform_normalSpeed", 0.2)
        self.speed_slow = settings.get("platform_slowDownSpeed", 0.1)
        self.move_front = settings.get("platform_move_front", 1)
        self.move_left = settings.get("platform_move_left", 1)
        self.move_right = settings.get("platform_move_right", 1)
        
        # Update obstacle thresholds
        stop_dist = settings.get("platform_stop_dist", 0.6)
        slow_dist = settings.get("platform_clear_dist", 1.5)
        self.obstacles.update_thresholds(stop_dist, slow_dist)
        

        print("Settings updated - Fast:", {self.speed_fast}," Slow:", {self.speed_slow})
        print("Stop distance:", {stop_dist}, "Slow distance:", {slow_dist})

        
    def distance_callback(self, msg):
        
        self.obstacles.update_distances(list(msg.data))
        
    def joystick_callback(self, data):
        
        # Print current distances for debugging

        #print(self.obstacles)

            
        if not self.override:
            self.process_movement(data)
            
    def process_movement(self, joy_data):
        twist = Twist()
        
        # Get joystick inputs
        linear_input = joy_data.axes[1] * 2  # Forward/backward
        angular_input = joy_data.axes[0]     # Left/right turning
        
        # Check for toggle (small vs large joystick)
        try:
            toggle_check = joy_data.axes[3]
        except:
            toggle_check = 10  # Default for small joystick
            
        if toggle_check == 10:
            angular_input = angular_input * -1  # Invert for small joystick
            
        # Apply movement restrictions
        if self.move_front == 0:
            linear_input = 0
        if (self.move_left == 0) and (angular_input > 0):
            angular_input = 0
        if (self.move_right == 0) and (angular_input < 0):
            angular_input = 0
            
        # Process movement based on toggle state
        if toggle_check == -1.0:  # Override mode - disable obstacle avoidance
            twist = self.move_without_obstacles(linear_input, angular_input, twist)
        else:  # Normal mode - with obstacle avoidance
            twist = self.move_with_obstacles(linear_input, angular_input, twist)
            
        # Publish command if not clamped
        if not self.clamp:
            self.cmd_pub.publish(twist)
        else:
            print("changed to see clamped") 
            
    def move_without_obstacles(self, linear_input, angular_input, twist):
        
        twist.linear.x = self.speed_fast * linear_input
        twist.angular.z = self.speed_fast * angular_input * 2.5
        return twist
        
    def move_with_obstacles(self, linear_input, angular_input, twist):
        
        # Forward movement
        if linear_input > 0.6 or abs(angular_input) > 0.8:  # Moving forward
            twist = self.move(linear_input,angular_input, twist)

        elif linear_input < 0:  # Moving backward  
            twist.linear.x = 0
            twist.angular.z = 0
            
        return twist
        
    def move(self, speed_input,angular_input, twist):
        
        if 3 in self.obstacles.flags :
            twist.linear.x = 0
            twist.angular.z = 0
        elif 2 in self.obstacles.flags :
            slow_factor = (self.obstacles.front_front - self.obstacles.dist_stop) / \
                         (self.obstacles.dist_slow - self.obstacles.dist_stop)
            slow_scale = (6 * slow_factor) / (1 + (6 * slow_factor))
            twist.linear.x = self.speed_fast * speed_input * slow_scale
            twist.angular.z = self.speed_slow * angular_input * 2.0
        else:
            twist.linear.x = self.speed_fast * speed_input
            twist.angular.z = self.speed_slow * angular_input * 2.5
        print(twist)
        return twist
        
    def emergency_stop(self):
       
        twist = Twist()  # All zeros
        self.cmd_pub.publish(twist)
        self.emergency_pub.publish(twist)
        print("Emergency stop activated")
        
    def trigger_callback(self, data):
        global clamp
        if data.data // 10 == 2:
          self.clamp = False
           
        elif data.data // 10 == 3 or data.data // 10 == 4:
           self.clamp = True
           print("clamped:",clamp)
           
    def settings_callback(self, data):
        
        try:
            settings = ast.literal_eval(data.data)
            self.update_settings(settings)
        except Exception as e:
            rospy.logerr("Settings update error: {e}")

            
    def override_callback(self, data):
        
        if data.data == 2:
            self.override = True
            
        elif data.data == 1:
            self.override = False
            
            
    def run(self):
        """Main run loop"""

        rospy.spin()

if __name__ == '__main__':
    clamp = True
    emergency = False
    override = False
    try:
        controller = JoystickController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Joystick Controller shutting down")
