import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class DroneNavigation(Node):
    def __init__(self):
        super().__init__('drone_navigation')
        
        # Initialize action client for navigation
        self._navigate_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribe to sensor data
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile_sensor_data
        )
        
        # Initialize a buffer for transforms (for pose tracking)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Target position for the drone (could come from user input or mission planner)
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'
        self.target_pose.pose.position.x = 10.0
        self.target_pose.pose.position.y = 5.0
        self.target_pose.pose.position.z = 3.0  # Assume drone's flying height
        
        # Store obstacles data from Lidar for dynamic re-planning
        self.obstacle_detected = False
        
        # Execute the navigation to target position
        self.navigate_to_target()

    def navigate_to_target(self):
        """Send the goal to the NavigateToPose action server"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.target_pose
        
        self.get_logger().info(f'Navigating to: {self.target_pose.pose.position.x}, {self.target_pose.pose.position.y}')
        self._navigate_action_client.wait_for_server()
        self._send_goal_future = self._navigate_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback when the drone reaches its goal."""
        result = future.result().result
        if result:
            self.get_logger().info('Goal reached!')
        else:
            self.get_logger().warn('Failed to reach goal')
    
    def lidar_callback(self, msg: LaserScan):
        """Lidar data processing for obstacle detection."""
        # Check for obstacles within a certain range (e.g., < 2 meters)
        min_range = min(msg.ranges)
        if min_range < 2.0:
            self.obstacle_detected = True
            self.get_logger().warn('Obstacle detected, re-planning route!')
            # Trigger re-planning here
            self.replan_path()

    def replan_path(self):
        """Replan the path when obstacles are detected."""
        # Here you could dynamically call a new path using the Nav2 path planning services
        # or adjust the current trajectory based on new sensor data
        self.get_logger().info('Re-planning path to avoid obstacle')

        # For now, we will stop navigating
        self._navigate_action_client.cancel_all_goals()

        # After detecting the obstacle, a new goal or trajectory should be recalculated

def main(args=None):
    rclpy.init(args=args)
    drone_navigation = DroneNavigation()
    rclpy.spin(drone_navigation)

    drone_navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
