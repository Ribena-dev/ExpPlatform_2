#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import time

class DualLidarProcessor:
    def __init__(self):
        rospy.init_node('dual_lidar_processor', anonymous=True)
        
        # Initialize variables
        self.front_ranges = None
        self.rear_ranges = None
        
        # Distance calculations
        self.dist_front = 0.0
        self.dist_left = 0.0  
        self.dist_right = 0.0
        self.dist_back = 0.0
        
        # Subscribers
        self.front_sub = rospy.Subscriber('/base_scan', LaserScan, self.front_lidar_callback)
        self.rear_sub = rospy.Subscriber('/rear_scan', LaserScan, self.rear_lidar_callback)
        
        # Publisher for distance array
        self.distance_pub = rospy.Publisher('/lidar_distances', Float32MultiArray, queue_size=1)
        
        # Publisher for individual distances (optional)
        self.front_pub = rospy.Publisher('/distance_front', Float32MultiArray, queue_size=1)
        self.rear_pub = rospy.Publisher('/distance_rear', Float32MultiArray, queue_size=1)
        
        # Timer for processing and publishing
        self.timer = rospy.Timer(rospy.Duration(0.1), self.process_and_publish)  # 10 Hz
        
        rospy.loginfo("Dual Lidar Processor initialized")
        rospy.loginfo("Publishing combined distances to: /lidar_distances")
        rospy.loginfo("Publishing front distances to: /distance_front") 
        rospy.loginfo("Publishing rear distances to: /distance_rear")
        
    def front_lidar_callback(self, msg):
        """Callback for front lidar data"""
        self.front_ranges = np.array(msg.ranges)
        
    def rear_lidar_callback(self, msg):
        """Callback for rear lidar data"""
        self.rear_ranges = np.array(msg.ranges)
        
    def calc_avg_distance(self, ranges):
        """Calculate average distance using percentile method (same as original code)"""
        if ranges is None or len(ranges) == 0:
            return 0.0
        
        # Filter out invalid readings (inf, nan, 0)
        valid_ranges = ranges[(ranges > 0.1) & (ranges < 30.0) & np.isfinite(ranges)]
        
        if len(valid_ranges) == 0:
            return 0.0
            
        # Use 50th percentile (median) same as original code
        return float(np.percentile(valid_ranges, 50))
    
    def process_front_lidar(self):
        """Process front lidar data to get front, left, and right distances"""
        if self.front_ranges is None:
            return [0.0, 0.0, 0.0, 0.0, 0.0]  # [left, front-left, front, front-right, right]
            
        total_points = len(self.front_ranges)
        
        # Divide into 5 sections like original code
        # Assuming 270° coverage divided into 5 sectors
        sector_size = total_points // 5
        
        # Calculate distances for each sector
        dist_right = self.calc_avg_distance(self.front_ranges[0:sector_size])
        dist_front_right = self.calc_avg_distance(self.front_ranges[sector_size:2*sector_size])
        dist_front = self.calc_avg_distance(self.front_ranges[2*sector_size:3*sector_size])
        dist_front_left = self.calc_avg_distance(self.front_ranges[3*sector_size:4*sector_size])
        dist_left = self.calc_avg_distance(self.front_ranges[4*sector_size:])
        
        return [dist_left, dist_front_left, dist_front, dist_front_right, dist_right]
    
    def process_rear_lidar(self):
        """Process rear lidar data to get back, left, and right distances"""
        if self.rear_ranges is None:
            return [0.0, 0.0, 0.0, 0.0, 0.0]  # [left, back-left, back, back-right, right]
            
        total_points = len(self.rear_ranges)
        
        # Divide into 5 sections
        sector_size = total_points // 5
        
        # For rear lidar, the orientation might be different
        # Adjust these based on your rear lidar mounting orientation
        dist_rear_left = self.calc_avg_distance(self.rear_ranges[0:sector_size])
        dist_back_left = self.calc_avg_distance(self.rear_ranges[sector_size:2*sector_size])
        dist_back = self.calc_avg_distance(self.rear_ranges[2*sector_size:3*sector_size])
        dist_back_right = self.calc_avg_distance(self.rear_ranges[3*sector_size:4*sector_size])
        dist_rear_right = self.calc_avg_distance(self.rear_ranges[4*sector_size:])
        
        return [dist_rear_left, dist_back_left, dist_back, dist_back_right, dist_rear_right]
    
    def process_and_publish(self, event):
        """Main processing function called by timer"""
        
        # Process both lidars
        front_distances = self.process_front_lidar()
        rear_distances = self.process_rear_lidar()
        
        # Extract key distances
        # Front lidar: [left, front-left, front, front-right, right]
        self.dist_front = front_distances[2]  # center front
        front_left = front_distances[0]       # left side from front lidar
        front_right = front_distances[4]      # right side from front lidar
        
        # Rear lidar: [rear-left, back-left, back, back-right, rear-right]  
        self.dist_back = rear_distances[2]    # center back
        rear_left = rear_distances[0]         # left side from rear lidar
        rear_right = rear_distances[4]        # right side from rear lidar
        
        # Combine left and right measurements (take minimum for safety)
        self.dist_left = min(front_left, rear_left) if front_left > 0 and rear_left > 0 else max(front_left, rear_left)
        self.dist_right = min(front_right, rear_right) if front_right > 0 and rear_right > 0 else max(front_right, rear_right)
        
        # Create and publish combined distance array
        # Format: [front, left, right, back]
        combined_distances = [self.dist_front, self.dist_left, self.dist_right, self.dist_back]
        self.publish_distances(combined_distances, self.distance_pub, 
                             ['front', 'left', 'right', 'back'])
        
        # Publish detailed front distances
        self.publish_distances(front_distances, self.front_pub,
                             ['left', 'front_left', 'front', 'front_right', 'right'])
        
        # Publish detailed rear distances  
        self.publish_distances(rear_distances, self.rear_pub,
                             ['rear_left', 'back_left', 'back', 'back_right', 'rear_right'])
        
        # Print for debugging
        rospy.loginfo_throttle(1.0, 
            f"Distances - Front: {self.dist_front:.2f}m, Left: {self.dist_left:.2f}m, "
            f"Right: {self.dist_right:.2f}m, Back: {self.dist_back:.2f}m")
    
    def publish_distances(self, distances, publisher, labels):
        """Publish distance array with labels"""
        msg = Float32MultiArray()
        
        # Set up the dimensions
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "distances"
        msg.layout.dim[0].size = len(distances)
        msg.layout.dim[0].stride = len(distances)
        msg.layout.data_offset = 0
        
        # Set the data
        msg.data = distances
        
        # Publish
        publisher.publish(msg)

    def run(self):
        """Main run loop"""
        rospy.loginfo("Starting dual lidar processor...")
        rospy.spin()

if __name__ == '__main__':
    try:
        processor = DualLidarProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Dual Lidar Processor shutting down")
