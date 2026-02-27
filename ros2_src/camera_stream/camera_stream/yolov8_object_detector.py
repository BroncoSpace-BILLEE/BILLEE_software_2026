#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class YOLOv8ObjectDetector(Node):
    def __init__(self):
        super().__init__("yolov8_object_detector")

        self.declare_parameter("input_topic", "/camera/video/compressed")
        self.declare_parameter("detection_topic", "/object_detected")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("model_path", "src/camera_stream/models/yolov8s.pt")
        self.declare_parameter("model_size", "s")
        self.declare_parameter("enable_gpu", True)
        self.declare_parameter(
            "target_classes",
            [],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description="List of target class names to detect"
            )
        )

        self.input_topic = self.get_parameter("input_topic").value
        self.detection_topic = self.get_parameter("detection_topic").value
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.model_path = str(self.get_parameter("model_path").value)
        self.model_size = str(self.get_parameter("model_size").value)
        self.enable_gpu = bool(self.get_parameter("enable_gpu").value)
        self.target_classes = self.get_parameter("target_classes").value

        if YOLO is None:
            self.get_logger().error(
                "ultralytics YOLO is not available. Install with: pip install ultralytics"
            )
            raise RuntimeError("YOLO not available")

        self.get_logger().info("Loading YOLO model...")
        if self.model_path:
            model_file = self.model_path
            if not os.path.isabs(model_file):
                package_share = self.get_package_share_directory('camera_stream')
                model_file = os.path.join(package_share, 'models', self.model_path)
            self.model = YOLO(model_file)
            self.get_logger().info(f"Loaded custom model from {model_file}")
        else:
            self.get_logger().info(f"Loading YOLOv8{self.model_size} model...")
            self.model = YOLO(f"yolov8{self.model_size}.pt")
        
        if self.enable_gpu:
            self.model.to("cuda")
        self.get_logger().info("YOLO model loaded")

        self.pub = self.create_publisher(Bool, self.detection_topic, 10)
        self.sub = self.create_subscription(
            CompressedImage, self.input_topic, self.on_image, 10
        )

        self.get_logger().info(
            f"Subscribing to {self.input_topic}, publishing to {self.detection_topic}"
        )

    def on_image(self, msg: CompressedImage):
        try:
            frame = cv2.imdecode(
                np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR
            )
            if frame is None:
                return

            results = self.model(frame, conf=self.confidence_threshold, verbose=False)

            object_detected = False
            for box in results[0].boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                
                if self.target_classes:
                    if class_name in self.target_classes:
                        object_detected = True
                        break
                else:
                    object_detected = True
                    break

            pub_msg = Bool()
            pub_msg.data = object_detected
            self.pub.publish(pub_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")


def main():
    rclpy.init()
    node = YOLOv8ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
