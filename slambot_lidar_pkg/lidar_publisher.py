#!/usr/bin/env python3

import sys
import argparse
import time
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time

from slambot_lidar_pkg.lidar import LidarStreamer

class LidarPublisher(Node):
    def __init__(self, lidar_port):
        super().__init__("lidar_publisher")
        self.get_logger().info("Running lidar publisher node")
        self.lidar_publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.lidar_streamer = LidarStreamer(port=lidar_port)
        time.sleep(1)
        self.start_scanner()

        self.scan_timer_ = self.create_timer(1.0, self.capture_scan)
        
    def start_scanner(self):
        self.lidar_streamer.start()
        time.sleep(2)

    def capture_scan(self):
        self.scan_data_ = self.lidar_streamer.get_latest_measurements()
        self.process_scan_ranges()
        self.publish_scan()

    def process_scan_ranges(self):
        self.ranges_ = []

        for _,distance in sorted(self.scan_data_.items(), key=lambda measurement:float(measurement[0])):
            distance_in_metres = distance / 1000

            self.ranges_.append(distance_in_metres if distance_in_metres > 0.0 else math.inf)

    def publish_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "laser"
        scan_msg.angle_min = math.radians(0)
        scan_msg.angle_max = math.radians(359)
        scan_msg.angle_increment = math.radians(0.72)
        scan_msg.range_min = 0.05
        scan_msg.range_max = 12.00
        scan_msg.ranges = self.ranges_

        self.lidar_publisher_.publish(scan_msg)

    def destroy_node(self):
        self.get_logger().info("Stopping Lidar Streamer")
        self.lidar_streamer.stop()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='LidarPublisher')
    parser.add_argument('--lidar_port', type=str, default='/dev/ttyUSB0', help='Set lidar port')

    user_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = LidarPublisher(lidar_port=user_args.lidar_port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Destroying node and stopping Lidar Streamer")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()