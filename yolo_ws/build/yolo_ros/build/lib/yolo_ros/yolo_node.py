#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.get_logger().info("Starting YOLOv11 ROS2 Node")

        # CV bridge
        self.bridge = CvBridge()

        # Load YOLOv11 model (auto-downloads)
        self.model = YOLO("yolo11m.pt")

        # Subscribe to OAK-D RGB camera
        self.sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )

        # Publish annotated image
        self.pub_img = self.create_publisher(
            Image,
            '/yolo/image_annotated',
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image → OpenCV safely
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )

            # OAK-D usually gives RGB, convert to BGR for OpenCV/YOLO
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # YOLO inference
            results = self.model(frame, verbose=False)[0]

            # Draw bounding boxes
            annotated = results.plot()

            # Convert back to ROS Image
            out_msg = self.bridge.cv2_to_imgmsg(
                annotated, encoding="bgr8"
            )
            out_msg.header = msg.header

            self.pub_img.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"YOLO callback error: {e}")


def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

