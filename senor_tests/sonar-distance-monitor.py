#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float32

class SonarDistanceMonitor:
    def __init__(self):
        # Initialize the node
        rospy.init_node('sonar_distance_monitor', anonymous=True)
        
        # Parameters
        self.num_closest = 5  # Number of closest readings to report
        self.latest_msg = None
        self.sensor_ids = []  # Will be populated if channel data contains sensor IDs
        
        # Subscribe to sonar topic
        self.sonar_sub = rospy.Subscriber(
            'RosAria/sonar',  # Change this if your topic is different
            PointCloud, 
            self.sonar_callback, 
            queue_size=10
        )
        
        # Publisher for minimum distance (optional - useful for other nodes)
        self.min_dist_pub = rospy.Publisher(
            '/sonar/min_distance', 
            Float32, 
            queue_size=5
        )
        
        rospy.loginfo("Sonar distance monitor initialized")
        rospy.loginfo("Monitoring for the %d closest sonar readings", self.num_closest)
        
        # Timer for periodic distance reporting
        rospy.Timer(rospy.Duration(0.5), self.report_distances)
        
    def calculate_distance(self, point):
        """Calculate Euclidean distance from origin to point (in 2D since Z is 0)"""
	if (point.x == 0.0 and point.y == 0.0):return -1
        return (math.sqrt(point.x**2 + point.y**2))
    
    def calculate_angle(self, point):
        """Calculate angle in degrees from robot's forward direction"""
	if  (point.x  == 0.0 and  point.y == 0.0) : return -1
        angle_rad = math.atan2(point.y, point.x)
        angle_deg = math.degrees(angle_rad)
        return angle_deg
        
    def sonar_callback(self, msg):
        """Store the latest sonar message"""
        self.latest_msg = msg
        
        # Extract sensor IDs from channels if available
        for channel in msg.channels:
            if channel.name.lower() in ['id', 'sensor_id', 'index']:
                self.sensor_ids = channel.values
                break
        
        # Immediately publish the minimum distance
        if msg.points:
            distances = [self.calculate_distance(point) for point in msg.points]
            min_dist = min(distances)
            self.min_dist_pub.publish(Float32(min_dist))
    
    def report_distances(self, event=None):
        """Report the closest distances from the latest sonar reading"""
        if not self.latest_msg:
            rospy.logwarn("No sonar data received yet")
            return
            
        if not self.latest_msg.points:
            rospy.logwarn("Received sonar message with no points")
            return
            
        # Calculate distances for all points
        distances = []
        for i, point in enumerate(self.latest_msg.points):
            distance = self.calculate_distance(point)
            angle = self.calculate_angle(point)
            
            # Determine sensor ID or index
            sensor_id = i  # Default to index if no channel data
            if i < len(self.sensor_ids):
                sensor_id = int(self.sensor_ids[i])
	    if (distance != -1):
                distances.append((distance, point, angle, sensor_id))
            
        # Sort by distance
        distances.sort(key=lambda x: x[0])
        
        # Get the closest N readings
        closest = distances[:min(self.num_closest, len(distances))]
        
        # Print header
        rospy.loginfo("\n--- 5 Closest Sonar Readings ---")
        rospy.loginfo("Rank | Sensor ID | Distance (m) | Angle (deg) | Position (x,y)")
        rospy.loginfo("-------------------------------------------------")
        
        # Print each close reading
        for rank, (distance, point, angle, sensor_id) in enumerate(closest, 1):
            rospy.loginfo(
                "%2d   | %9d | %11.3f | %10.1f | (%.2f, %.2f)", 
                rank, sensor_id, distance, angle, point.x, point.y
            )
            
        # Print footer with warning if needed
        rospy.loginfo("-------------------------------------------------")
        if closest[0][0] < 0.5:  # Warning threshold (adjust as needed)
            rospy.logwarn("WARNING: Object detected at %.2f meters!", closest[0][0])
            
    def run(self):
        """Run the node"""
        rospy.loginfo("Sonar distance monitor running. Press Ctrl+C to stop.")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down sonar distance monitor")

if __name__ == '__main__':
    try:
        monitor = SonarDistanceMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
