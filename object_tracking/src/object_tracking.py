#!/usr/bin/env python3

from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from tf2_ros import TransformBroadcaster


DEBUG_IMAGE_DIR = Path("/tmp/object_tracking_debug")
DEBUG_IMAGE_PATH = DEBUG_IMAGE_DIR / "latest_image.ppm"


class ObjectTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__("object_tracking_server")

        self.image_topic = (
            self.declare_parameter("image_topic", "/camera/frame_rgb")
            .get_parameter_value()
            .string_value
        )
        self.pointcloud_topic = (
            self.declare_parameter("pointcloud_topic", "/camera/frame_pc")
            .get_parameter_value()
            .string_value
        )

        self.latest_image: Optional[Image] = None
        self.latest_pointcloud: Optional[PointCloud2] = None
        self.has_new_image = False

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.handle_image,
            qos_profile_sensor_data,
        )
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.handle_pointcloud,
            qos_profile_sensor_data,
        )
        self.target_transform_broadcaster = TransformBroadcaster(self)
        self.update_timer = self.create_timer(0.1, self.update_pose)

        self.get_logger().info(f"Subscribing to camera images on {self.image_topic}")
        self.get_logger().info(f"Subscribing to point cloud on {self.pointcloud_topic}")
        self.get_logger().info("Target transforms will be broadcast on /tf")
        self.get_logger().info(f"Debug images will be saved to {DEBUG_IMAGE_PATH}")

    def handle_image(self, msg: Image) -> None:
        self.latest_image = msg
        self.has_new_image = True

    def handle_pointcloud(self, msg: PointCloud2) -> None:
        self.latest_pointcloud = msg

    def update_pose(self) -> None:
        if self.latest_image is None or not self.has_new_image:
            return

        image = self.latest_image
        self.has_new_image = False
        self.save_debug_image(image)

    def save_debug_image(self, image: Image) -> None:
        width = image.width
        height = image.height
        row_bytes = width * 3

        DEBUG_IMAGE_DIR.mkdir(parents=True, exist_ok=True)
        with DEBUG_IMAGE_PATH.open("wb") as image_file:
            image_file.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
            for y in range(height):
                row_start = y * image.step
                row = image.data[row_start : row_start + row_bytes]
                rgb_row = bytearray(row_bytes)
                for x in range(width):
                    pixel = x * 3
                    rgb_row[pixel] = row[pixel + 2]
                    rgb_row[pixel + 1] = row[pixel + 1]
                    rgb_row[pixel + 2] = row[pixel]
                image_file.write(rgb_row)

    def publish_target_transform(self, transform: TransformStamped) -> None:
        self.target_transform_broadcaster.sendTransform(transform)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ObjectTrackingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
