#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Subscribe to the IMU topic
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        # Initialize variables for storing orientation
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def imu_callback(self, msg):
        """Callback function to process IMU data."""
        orientation_q = msg.orientation

        # Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
        self.roll, self.pitch, self.yaw = self.quaternion_to_euler(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

        # Log the orientation for debugging purposes
        self.get_logger().info(f'Roll: {self.roll:.2f}, Pitch: {self.pitch:.2f}, Yaw: {self.yaw:.2f}')

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Quaternion to roll
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # Quaternion to pitch
        t2 = +2.0 * (w * y - z * x)
        t2 = max(-1.0, min(t2, +1.0))  # Clamp t2 to the range [-1, 1]
        pitch = math.asin(t2)

        # Quaternion to yaw
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    
    # Cleanup after the node is stopped
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
