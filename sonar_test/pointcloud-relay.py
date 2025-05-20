#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

class PointCloudRelay:
    def __init__(self):
        # Get parameters
        input_topic = rospy.get_param('~input_topic', 'input/pointcloud')
        output_topic = rospy.get_param('~output_topic', 'output/pointcloud')
        
        # Create publisher and subscriber
        self.pub = rospy.Publisher(output_topic, PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber(input_topic, PointCloud2, self.callback)
        
        rospy.loginfo("PointCloud relay initialized")
        rospy.loginfo("Subscribing to: %s", input_topic)
        rospy.loginfo("Publishing to: %s", output_topic)
    
    def callback(self, msg):
        # Simply republish the message
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('pointcloud_relay')
    relay = PointCloudRelay()
    rospy.spin()
