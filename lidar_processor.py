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
        
        
        # Timer for processing and publishing
        self.timer = rospy.Timer(rospy.Duration(0.1), self.process_and_publish)  # 10 Hz
        
        print("Dual Lidar Processor initialized")
        
    def front_lidar_callback(self, msg):
        self.front_ranges = np.array(msg.ranges)
        
    def rear_lidar_callback(self, msg):
        self.rear_ranges = np.array(msg.ranges)
        
    def calc_avg_distance(self, ranges):
  
        if ranges is None or len(ranges) == 0:
            return 0.0
       
        return float(np.percentile(ranges, 50))
    
    def process_lidar(self,ranges):
        if ranges is None:
            return [0.0, 0.0, 0.0, 0.0, 0.0]  # [left, front-left, front, front-right, right]
        ranges = ranges[(ranges > 0.1) & (ranges < 30.0) & np.isfinite(ranges)]   
        total_points = len(ranges)
        sector_size = total_points // 5
        
        # Calculate distances for each sector
        dist_right = self.calc_avg_distance(ranges[0:sector_size])
        dist_front_right = self.calc_avg_distance(ranges[sector_size:2*sector_size])
        dist_front = self.calc_avg_distance(ranges[2*sector_size:3*sector_size])
        dist_front_left = self.calc_avg_distance(ranges[3*sector_size:4*sector_size])
        dist_left = self.calc_avg_distance(ranges[4*sector_size:])
        
        return [dist_left, dist_front_left, dist_front, dist_front_right, dist_right]
    
    
    def process_and_publish(self, event):        
        # Process both lidars
        front_distances = self.process_lidar(self.front_ranges)
        rear_distances = self.process_lidar(self.rear_ranges)
        
        combined_distances = front_distances + rear_distances
        self.publish_distances(combined_distances, self.distance_pub, 
                             ['front', 'back'])
        
        print(front_distances, rear_distances)
    
    def publish_distances(self, distances, publisher, labels):
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
        print("Starting dual lidar processor...")
        rospy.spin()

if __name__ == '__main__':
    try:
        processor = DualLidarProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        print("Dual Lidar Processor shutting down")
