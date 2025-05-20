#!/usr/bin/env python
import math 
import numpy as np
import rospy
from sensor_msgs.msg import Range , PointCloud
from std_msgs.msg import Float32MultiArray

class SonarProcessor:
    def __init__(self):
        rospy.init_node('sonar_processor', anonymous=True)
        
        # Subscribe to the RosAria sonar topic
        rospy.Subscriber('/RosAria/sonar', PointCloud , self.sonar_callback)
        
        # Initialize variables to store minimum distances
        self.back_left_min = float('inf')
        self.back_right_min = float('inf')
        
        rospy.loginfo("Sonar processor initialized")
    
   
 
    def sonar_callback(self, data):
        # Extract the ranges array
	#self.print_sonar(data.points[1:9])
        
        # Process the data and update minimum distances
        self.back_right_center_min,self.back_right_side_min,self.back_left__center_min, self.back_left_side_min = self.process_sonar_data(data)
        
        # Log the minimum distances
        #rospy.loginfo("Back Left Min: %.2f, Back Right Min: %.2f", 
        #             self.back_left_min, self.back_right_min)
    
    def print_sonar(self,data):
        for i in range(len(data)):
            print(i,data[i])
        return

    def process_sonar_data(self, ranges):
        """
        Process sonar data and return minimum distances for back-left and back-right regions.
      'inf')"""
        
        # front_right_ranges = ranges[0:8]  # indices 0-8 
        # front_left_ranges = ranges[8:16]  # indices 8-16
        back_right_side_ranges = ranges.points[16:19]  # indices 16-23 sonar index 16 starts at coner
        back_left_side_ranges = ranges.points[28:31]
        back_right_center_ranges = ranges.points[19:23]  # indices 16-23 sonar index 16 starts at coner
        back_left_center_ranges = ranges.points[24:28]  # indices 24-31 sonar index 24 starts from center
        back_r_center_dist = self.convert_distances(back_right_center_ranges)
        back_r_side_dist = self.convert_distances(back_right_side_ranges)
        back_l_center_dist = self.convert_distances(back_left_center_ranges)
        back_l_side_dist = self.convert_distances(back_left_side_ranges)
        print("br_center_dist")
        self.print_sonar(back_r_center_dist)
        print("br_side_dist")
        self.print_sonar(back_r_side_dist)
        print("bl__center_dist")
        self.print_sonar(back_l_center_dist)
        print("bl_side_dist")
        self.print_sonar(back_l_side_dist)
        brc_min = min(back_r_center_dist)
        brs_min = min(back_r_side_dist)
        blc_min = min(back_l_center_dist)
        bls_min = min(back_l_side_dist)
        print("back_right_center_min:",brc_min,"back_right_side_min:",brs_min,"back_left_center_min:",blc_min,"back_left_side_min:",bls_min)
        sonar_min_dist = Float32MultiArray()
        sonar_min_dist.data = [brc_min,brs_min,blc_min,bls_min]
        self.pub_dists(sonar_min_dist)
        return brc_min,brs_min,blc_min,bls_min
    
    def convert_distances(self,array):
        distances = []
        for point in array:
            distance = math.sqrt(point.x**2 + point.y**2)
            distances.append(distance)
        return distances

    def get_back(self):
        return self.brc_min,self.brs_min,self.blc_min,self.bls_min
    def pub_dists(self,min_distances):
        print(min_distances)
        pub = rospy.Publisher('/sonar_min',Float32MultiArray, queue_size=10)
        pub.publish(min_distances)


def run():
    processor = SonarProcessor()
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        # Get the minimum distances
        #bl_min, br_min = processor.get_back()#get back min dist
        
        # You can use the minimum distances here as needed
        # For example, print them or use them for decision making
        
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
