import rclpy
from rclpy.node import Node
import cv2
import onnxruntime as ort
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class YoloV11Node(Node):
    def __init__(self):
        super().__init__('yolov11_node')

        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            'camera_topic',
            self.image_callback,
            10)

        # Load the YOLOv11 ONNX model
        model_path = "/home/jawhar/ros2_ws_drone/src/gps_package/gps_package/YOLO11.onnx"
        self.session = ort.InferenceSession(model_path)

        # Initialize the CvBridge for ROS2 Image message conversion
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS2 Image message to OpenCV format
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Preprocess the image for YOLOv11
        input_image = self.preprocess_image(img)
        
        # Perform inference
        try:
            # Updated the input feed to match the expected input name ('images')
            outputs = self.session.run(None, {'images': input_image.astype(np.float32)})
            self.get_logger().info(f"Model outputs: {outputs}")
            # Post-process and publish the detected objects
            self.publish_results(outputs)
        except Exception as e:
            self.get_logger().error(f"Inference failed: {str(e)}")

    def preprocess_image(self, img):
        # Resize and normalize the image (YOLOv11 expects specific input dimensions)
        img_resized = cv2.resize(img, (2048, 2048))  # Adjust based on your model input size
        img_normalized = img_resized / 255.0  # Normalize to [0, 1]
        img_transposed = img_normalized.transpose(2, 0, 1)  # Adjust channel dimensions
        return np.expand_dims(img_transposed, axis=0)  # Add batch dimension

    def publish_results(self, outputs):
        # YOLO outputs typically include bounding boxes, class scores, and confidence levels
        output_data = outputs[0]  # Assuming batch size of 1, access the first output
        boxes = []  # List to store bounding boxes
        confidences = []  # List to store confidence scores
        class_ids = []  # List to store class IDs

        for detection in output_data:
            # Extract bounding box and class information
            box = detection[:4]  # First 4 values correspond to x, y, width, height
            confidence = detection[4]  # Objectness score
            class_scores = detection[5:]  # Class probabilities
            
            # Get the class with the highest score
            class_id = np.argmax(class_scores)
            class_confidence = class_scores[class_id]

            # Filter out detections based on confidence threshold
            if confidence > 0.5:  # Adjust threshold as needed
                boxes.append(box)
                confidences.append(confidence)
                class_ids.append(class_id)

        # Print detected boxes, confidences, and class IDs for debugging
        self.get_logger().info(f"Detected boxes: {boxes}")
        self.get_logger().info(f"Confidence scores: {confidences}")
        self.get_logger().info(f"Class IDs: {class_ids}")

        # Here you can further apply Non-Max Suppression (NMS) if needed
        # After processing, you can publish the results or visualize them on the image.

def main(args=None):
    rclpy.init(args=args)
    node = YoloV11Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
