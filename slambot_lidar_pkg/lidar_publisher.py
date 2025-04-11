#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from slambot_lidar_pkg.lidar import LidarStreamer

class LidarPublisherNode(Node):
    def __init__(self):
        super().__init__("lidar_publisher")
        self.get_logger().info("Running lidar publisher node")
        self.lidar_streamer = LidarStreamer(port="/dev/ttyUSB0")
        self.start_scan()

    def start_scan(self):
        scan = self.lidar_streamer.start()
        time.sleep(2)
        scan_data = self.lidar_streamer.get_latest_measurements()
        self.get_logger().info(f'Scan data: {scan_data}')
        self.lidar_streamer.stop()

def main(args=None):
    rclpy.init(args=None)
    node = LidarPublisherNode()
    rclpy.spin(node)
    node.lidar_streamer.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()