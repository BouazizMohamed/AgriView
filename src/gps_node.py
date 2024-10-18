#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import random
import time

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)
        self.timer = self.create_timer(1.0, self.publish_gps_data)

    def publish_gps_data(self):
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_frame"
        gps_msg.latitude = random.uniform(-90.0, 90.0)  # Simulated latitude
        gps_msg.longitude = random.uniform(-180.0, 180.0)  # Simulated longitude
        gps_msg.altitude = random.uniform(0.0, 5000.0)  # Simulated altitude
        gps_msg.status.status = 0  # Status of the GPS fix (0: no fix, 1: 2D fix, 2: 3D fix)
        gps_msg.status.service = 1  # Service type
        
        self.publisher_.publish(gps_msg)
        self.get_logger().info(f'Publishing GPS Data: {gps_msg.latitude}, {gps_msg.longitude}')

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
