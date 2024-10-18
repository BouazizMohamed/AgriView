#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class DroneFlightControlNode(Node):

    def __init__(self):
        super().__init__('drone_flight_control_node')

        # Subscribing to GPS, IMU, and velocity command topics
        self.gps_subscriber = self.create_subscription(NavSatFix, 'gps_topic', self.gps_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu_topic', self.imu_callback, 10)
        self.velocity_subscriber = self.create_subscription(Twist, 'cmd_vel', self.velocity_callback, 10)

        # Publishing motor commands
        self.motor_publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)

        # Internal variables for storing sensor data
        self.gps_data = None
        self.imu_data = None
        self.velocity_command = None

        # Timer to control update rate for motor commands
        self.timer = self.create_timer(0.1, self.control_loop)

    def gps_callback(self, msg: NavSatFix):
        self.get_logger().info(f"GPS Data: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}")
        self.gps_data = msg

    def imu_callback(self, msg: Imu):
        self.get_logger().info(f"IMU Data: orientation=({msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}), "
                               f"angular_velocity=({msg.angular_velocity.x}, {msg.angular_velocity.y}, {msg.angular_velocity.z})")
        self.imu_data = msg

    def velocity_callback(self, msg: Twist):
        self.get_logger().info(f"Velocity Command: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), "
                               f"angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})")
        self.velocity_command = msg

    def control_loop(self):
        if self.gps_data and self.imu_data and self.velocity_command:
            # Log input values
            self.get_logger().info("Input Values:")
            self.get_logger().info(f"  GPS Data: lat={self.gps_data.latitude}, lon={self.gps_data.longitude}, alt={self.gps_data.altitude}")
            self.get_logger().info(f"  IMU Data: orientation=({self.imu_data.orientation.x}, {self.imu_data.orientation.y}, {self.imu_data.orientation.z}), "
                                   f"angular_velocity=({self.imu_data.angular_velocity.x}, {self.imu_data.angular_velocity.y}, {self.imu_data.angular_velocity.z})")
            self.get_logger().info(f"  Velocity Command: linear=({self.velocity_command.linear.x}, {self.velocity_command.linear.y}, {self.velocity_command.linear.z}), "
                                   f"angular=({self.velocity_command.angular.x}, {self.velocity_command.angular.y}, {self.velocity_command.angular.z})")
            
            motor_commands = self.compute_motor_commands()
            motor_msg = Float32MultiArray()
            motor_msg.data = motor_commands
            self.motor_publisher.publish(motor_msg)
            # Log output values
            self.get_logger().info(f"Motor Commands: {motor_commands}")
        else:
            self.get_logger().warn("Waiting for all inputs (GPS, IMU, velocity command)...")

    def compute_motor_commands(self):
        # Assuming a quadcopter with four motors:
        # motor[0] - front-left
        # motor[1] - front-right
        # motor[2] - rear-left
        # motor[3] - rear-right

        # Extract velocity command (linear velocity in X, Y, Z, and angular velocities)
        linear_x = self.velocity_command.linear.x
        linear_y = self.velocity_command.linear.y
        linear_z = self.velocity_command.linear.z

        angular_roll = self.velocity_command.angular.x
        angular_pitch = self.velocity_command.angular.y
        angular_yaw = self.velocity_command.angular.z

        # Extract IMU data (orientation and angular velocities)
        roll = self.imu_data.orientation.x  # Roll (rotation around X-axis)
        pitch = self.imu_data.orientation.y  # Pitch (rotation around Y-axis)
        yaw = self.imu_data.orientation.z  # Yaw (rotation around Z-axis)
        
        # Simple proportional control gains
        k_linear = 1.0  # Gain for linear velocity control
        k_angular = 0.5  # Gain for angular control (roll, pitch, yaw)
        
        # Calculate motor thrust for each motor (a quadcopter with X configuration)
        motor_commands = [0.0, 0.0, 0.0, 0.0]
        
        # Linear velocity control (for altitude control)
        thrust = k_linear * linear_z
        
        # Angular control for stabilization (roll, pitch, yaw)
        motor_commands[0] = thrust - (k_angular * pitch) + (k_angular * roll) - (k_angular * yaw)  # front-left
        motor_commands[1] = thrust - (k_angular * pitch) - (k_angular * roll) + (k_angular * yaw)  # front-right
        motor_commands[2] = thrust + (k_angular * pitch) + (k_angular * roll) + (k_angular * yaw)  # rear-left
        motor_commands[3] = thrust + (k_angular * pitch) - (k_angular * roll) - (k_angular * yaw)  # rear-right

        # Normalize motor commands (ensure values are within allowed range, e.g., 0.0 to 1.0 for motor thrust)
        motor_commands = [min(max(motor, 0.0), 1.0) for motor in motor_commands]
        
        return motor_commands


def main(args=None):
    rclpy.init(args=args)
    node = DroneFlightControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
