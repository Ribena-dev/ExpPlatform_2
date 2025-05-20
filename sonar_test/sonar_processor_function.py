#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range

class SonarProcessor:
    def __init__(self):
        rospy.init_node('sonar_processor', anonymous=True)
        
        # Subscribe to the RosAria sonar topic
        rospy.Subscriber('/RosAria/sonar', Range, self.sonar_callback)
        
        # Initialize variables to store minimum distances
        self.back_left_min = float('inf')
        self.back_right_min = float('inf')
        
        rospy.loginfo("Sonar processor initialized")
    
    def sonar_callback(self, data):
        # Extract the ranges array
        ranges = data.ranges
        
        # Process the data and update minimum distances
        self.back_left_min, self.back_right_min = self.process_sonar_data(ranges)
        
        # Log the minimum distances
        rospy.loginfo("Back Left Min: %.2f, Back Right Min: %.2f", 
                     self.back_left_min, self.back_right_min)
    
    def process_sonar_data(self, ranges):
        """
        Process sonar data and return minimum distances for back-left and back-right regions.
        
        Args:
            ranges: List of sonar range measurements
            
        Returns:
            tuple: (back_left_min, back_right_min) as float values
        """
        # Ensure array has enough elements
        if len(ranges) < 32:
            rospy.logwarn("Received sonar array with insufficient elements: %d", len(ranges))
            return float('inf'), float('inf')
        
        # Extract regions (using 0-based indexing)
        # front_right_ranges = ranges[0:8]  # indices 1-8 in 1-based indexing
        # front_left_ranges = ranges[8:16]  # indices 9-16 in 1-based indexing
        back_left_ranges = ranges[16:24]  # indices 17-24 in 1-based indexing
        back_right_ranges = ranges[24:32]  # indices 25-32 in 1-based indexing
        
        # Find minimum values for back regions
        back_left_min = min(back_left_ranges)
        back_right_min = min(back_right_ranges)
        
        return back_left_min, back_right_min
    
    def get_back_min_distances(self):
        """
        Get the minimum distances for back-left and back-right regions.
        
        Returns:
            tuple: (back_left_min, back_right_min) as float values
        """
        return self.back_left_min, self.back_right_min

def run():
    processor = SonarProcessor()
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        # Get the minimum distances
        bl_min, br_min = processor.get_back_min_distances()
        
        # You can use the minimum distances here as needed
        # For example, print them or use them for decision making
        
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
