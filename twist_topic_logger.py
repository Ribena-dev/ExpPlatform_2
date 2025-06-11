#!/usr/bin/env python
"""
twist_logger.py
Standalone CSV logger that subscribes to twist topics
Run this separately from your joystick controller
"""

import rospy
import csv
from datetime import datetime
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

class TwistTopicLogger:
    def __init__(self):
        rospy.init_node('twist_logger', anonymous=True)
        
        self.logging = False
        self.csv_file = None
        self.csv_writer = None
        self.start_time = None
        self.filename = None
        
        # Subscribe to twist commands
        self.twist_sub = rospy.Subscriber('RosAria/cmd_vel', Twist, self.twist_callback)
        
        # Subscribe to trigger messages for auto start/stop
        self.trigger_sub = rospy.Subscriber('trigger_msgs', Int16, self.trigger_callback)
        
        rospy.loginfo("Twist Logger initialized")
        rospy.loginfo("Subscribed to: RosAria/cmd_vel and trigger_msgs")
        rospy.loginfo("Commands: 'start', 'stop', 'status', 'quit'")
        
    def twist_callback(self, msg):
        """Log twist messages when logging is active"""
        if not self.logging or not self.csv_writer:
            return
            
        try:
            current_time = rospy.Time.now().to_sec() - self.start_time
            self.csv_writer.writerow([current_time, msg.linear.x, msg.angular.z])
            self.csv_file.flush()  # Ensure data is written immediately
        except Exception as e:
            rospy.logerr(f"Error logging twist: {e}")
        
    def trigger_callback(self, msg):
        """Auto start/stop logging based on trigger messages"""
        if msg.data // 10 == 2:  # Experiment start
            self.start_logging()
        elif msg.data // 10 == 3 or msg.data // 10 == 4:  # Experiment end
            self.stop_logging()
        
    def start_logging(self, filename=None):
        """Start logging to CSV file"""
        if self.logging:
            print("Already logging!")
            return
            
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"twist_data_{timestamp}.csv"
            
        try:
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Write header
            self.csv_writer.writerow(['time', 'linear_x', 'angular_z'])
            
            self.logging = True
            self.start_time = rospy.Time.now().to_sec()
            self.filename = filename
            print(f"Started logging to {filename}")
            rospy.loginfo(f"Started logging twist commands to {filename}")
            
        except Exception as e:
            rospy.logerr(f"Error starting logging: {e}")
        
    def stop_logging(self):
        """Stop logging and close file"""
        if not self.logging:
            print("Not currently logging!")
            return
            
        try:
            self.logging = False
            if self.csv_file:
                self.csv_file.close()
                print(f"Stopped logging. Data saved to {self.filename}")
                rospy.loginfo(f"Stopped logging. Data saved to {self.filename}")
            self.csv_file = None
            self.csv_writer = None
            self.filename = None
        except Exception as e:
            rospy.logerr(f"Error stopping logging: {e}")
            
    def get_status(self):
        """Get current logging status"""
        if self.logging:
            duration = rospy.Time.now().to_sec() - self.start_time
            print(f"Currently logging to {self.filename} (duration: {duration:.1f}s)")
        else:
            print("Not currently logging")
            
    def run(self):
        """Main run loop with command interface"""
        print("\n=== Twist Logger Commands ===")
        print("- 'start' : Start logging manually")
        print("- 'stop'  : Stop logging manually")
        print("- 'status': Check logging status")
        print("- 'quit'  : Exit logger")
        print("- Logging auto-starts/stops with trigger messages")
        print("")
        
        # Command interface in separate thread
        import threading
        
        def command_interface():
            while not rospy.is_shutdown():
                try:
                    cmd = input().strip().lower()
                    if cmd == 'start':
                        self.start_logging()
                    elif cmd == 'stop':
                        self.stop_logging()
                    elif cmd == 'status':
                        self.get_status()
                    elif cmd == 'quit':
                        if self.logging:
                            self.stop_logging()
                        rospy.signal_shutdown("User quit")
                        break
                    elif cmd == '':
                        continue  # Ignore empty input
                    else:
                        print("Unknown command. Use: start, stop, status, quit")
                except KeyboardInterrupt:
                    if self.logging:
                        self.stop_logging()
                    rospy.signal_shutdown("Keyboard interrupt")
                    break
                except:
                    break
                    
        cmd_thread = threading.Thread(target=command_interface)
        cmd_thread.daemon = True
        cmd_thread.start()
        
        # Keep the node alive
        try:
            rospy.spin()
        except KeyboardInterrupt:
            if self.logging:
                self.stop_logging()

if __name__ == '__main__':
    try:
        logger = TwistTopicLogger()
        logger.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Twist Logger shutting down")