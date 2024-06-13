#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math

class LaserFilterNode:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.left_scan_pub = rospy.Publisher('/scan_left_side', LaserScan, queue_size=1)
        self.right_scan_pub = rospy.Publisher('/scan_right_side', LaserScan, queue_size=1)

        # Define the angle range for filtering
        self.left_min_angle = -0.8
        self.left_max_angle = 0.8
        self.right_min_angle_1 = -3.2
        self.right_max_angle_1 = -2.4
        self.right_min_angle_2 = 2.4
        self.right_max_angle_2 = 3.2

    def laser_callback(self, scan_msg):
        left_scan = self.create_filtered_scan(scan_msg, self.left_min_angle, self.left_max_angle)
        right_scan = self.create_filtered_scan(scan_msg, self.right_min_angle_1, self.right_max_angle_1, self.right_min_angle_2, self.right_max_angle_2)

        self.left_scan_pub.publish(left_scan)
        self.right_scan_pub.publish(right_scan)

    def create_filtered_scan(self, scan_msg, min_angle_1, max_angle_1, min_angle_2=None, max_angle_2=None):
        filtered_scan = LaserScan()
        filtered_scan.header = scan_msg.header
        filtered_scan.angle_min = scan_msg.angle_min
        filtered_scan.angle_max = scan_msg.angle_max
        filtered_scan.angle_increment = scan_msg.angle_increment
        filtered_scan.time_increment = scan_msg.time_increment
        filtered_scan.scan_time = scan_msg.scan_time
        filtered_scan.range_min = scan_msg.range_min
        filtered_scan.range_max = scan_msg.range_max
        filtered_scan.ranges = list(scan_msg.ranges)
        filtered_scan.intensities = list(scan_msg.intensities)

        # Calculate the indices corresponding to the desired angle ranges
        min_index_1 = int((min_angle_1 - scan_msg.angle_min) / scan_msg.angle_increment)
        max_index_1 = int((max_angle_1 - scan_msg.angle_min) / scan_msg.angle_increment)
        
        if min_angle_2 is not None and max_angle_2 is not None:
            min_index_2 = int((min_angle_2 - scan_msg.angle_min) / scan_msg.angle_increment)
            max_index_2 = int((max_angle_2 - scan_msg.angle_min) / scan_msg.angle_increment)
        else:
            min_index_2 = None
            max_index_2 = None

        # Set ranges outside the desired angle ranges to infinity (or any value indicating no detection)
        for i in range(len(filtered_scan.ranges)):
            if not (min_index_1 <= i <= max_index_1 or (min_index_2 is not None and max_index_2 is not None and min_index_2 <= i <= max_index_2)):
                filtered_scan.ranges[i] = float('inf')
                filtered_scan.intensities[i] = 0.0

        return filtered_scan

if __name__ == '__main__':
    rospy.init_node('laser_filter_node')
    node = LaserFilterNode()
    rospy.spin()
