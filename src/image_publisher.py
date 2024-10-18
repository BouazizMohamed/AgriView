import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self, image_path):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.image_path = image_path
        self.bridge = CvBridge()

    def timer_callback(self):
        # Read the image using OpenCV
        cv_image = cv2.imread(self.image_path)
        
        if cv_image is not None:
            # Convert OpenCV image to ROS2 Image message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            # Publish the image message
            self.publisher_.publish(ros_image)
            self.get_logger().info('Publishing image')
        else:
            self.get_logger().error(f"Could not read the image from {self.image_path}")

def main(args=None):
    rclpy.init(args=args)
    image_path = "/home/jawhar/ros2_ws_drone/src/gps_package/gps_package/Image.jpg"  # Set the absolute path of the image
    node = ImagePublisher(image_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
