#!/usr/bin/env python3

import importlib.util
import os
from pathlib import Path
from threading import Event, Lock, Thread
from time import sleep
from typing import Optional

os.environ["YOLO_AUTOINSTALL"] = "false"
os.environ["YOLO_OFFLINE"] = "true"

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from robotino_vision_msgs.srv import ToggleObjectTracking
from sensor_msgs.msg import Image, PointCloud2
from tf2_ros import TransformBroadcaster
from ultralytics.utils import downloads as ultralytics_downloads
from ultralytics import YOLOE


DEBUG_IMAGE_DIR = Path("/tmp/object_tracking_debug")
DEBUG_IMAGE_PATH = DEBUG_IMAGE_DIR / "latest_image.ppm"
MODEL_PATH = Path(__file__).resolve().parents[1] / "yoloe-26n-seg.pt"


def block_ultralytics_downloads(*args, **kwargs):
    url = kwargs.get("url", args[0] if args else "unknown URL")
    raise RuntimeError(f"Ultralytics attempted to download {url}; install all model assets manually")


ultralytics_downloads.safe_download = block_ultralytics_downloads


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
        self.latest_segmentation_map: Optional[np.ndarray] = None
        self.tracking_active = False
        self.current_object_prompt = ""
        self.data_lock = Lock()
        self.stop_update_loop = Event()
        self.model: Optional[YOLOE] = None
        self.model_object_prompt = ""

        self.sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sensor_callback_group = ReentrantCallbackGroup()

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.handle_image,
            self.sensor_qos,
            callback_group=self.sensor_callback_group,
        )
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.handle_pointcloud,
            self.sensor_qos,
            callback_group=self.sensor_callback_group,
        )
        self.object_tracking_service = self.create_service(
            ToggleObjectTracking,
            "object_tracking",
            self.handle_object_tracking,
            callback_group=self.sensor_callback_group,
        )
        self.target_transform_broadcaster = TransformBroadcaster(self)
        self.update_thread = Thread(
            target=self.update_pose,
            name="object_tracking_update_pose",
            daemon=True,
        )
        self.update_thread.start()

        self.get_logger().info(f"Subscribing to camera images on {self.image_topic}")
        self.get_logger().info(f"Subscribing to point cloud on {self.pointcloud_topic}")
        self.get_logger().info("Object tracking service ready on object_tracking")
        self.get_logger().info("Target transforms will be broadcast on /tf")
        self.get_logger().info(f"Debug images will be saved to {DEBUG_IMAGE_PATH}")

    def handle_image(self, msg: Image) -> None:
        with self.data_lock:
            self.latest_image = msg

    def handle_pointcloud(self, msg: PointCloud2) -> None:
        with self.data_lock:
            self.latest_pointcloud = msg

    def handle_object_tracking(
        self,
        request: ToggleObjectTracking.Request,
        response: ToggleObjectTracking.Response,
    ) -> ToggleObjectTracking.Response:
        with self.data_lock:
            self.tracking_active = request.enable
            if request.enable:
                self.current_object_prompt = request.object_prompt

        response.success = True
        response.error = ""
        return response

    def update_pose(self) -> None:
        while rclpy.ok() and not self.stop_update_loop.is_set():
            with self.data_lock:
                tracking_active = self.tracking_active
                image = self.latest_image
                object_prompt = self.current_object_prompt

            if not tracking_active or image is None:
                sleep(0.01)
                continue

            self.save_debug_image(image)
            segmentation_map = self.create_segmentation_map(image, object_prompt)
            with self.data_lock:
                self.latest_segmentation_map = segmentation_map

    def destroy_node(self) -> bool:
        self.stop_update_loop.set()
        if self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
        return super().destroy_node()

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

    def create_segmentation_map(self, image: Image, object_prompt: str) -> np.ndarray:
        if self.model is None:
            if not Path(MODEL_PATH).is_file():
                raise FileNotFoundError(f"YOLOE model file not found: {MODEL_PATH}")
            if importlib.util.find_spec("clip") is None:
                raise ModuleNotFoundError(
                    "YOLOE text prompts require the 'clip' Python package. "
                    "Install it manually before running this node."
                )
            self.model = YOLOE(MODEL_PATH)

        if object_prompt != self.model_object_prompt:
            self.model.set_classes([object_prompt])
            self.model_object_prompt = object_prompt

        frame = self.image_to_bgr(image)
        results = self.model(frame, verbose=False)
        segmentation_map = np.zeros((image.height, image.width), dtype=np.uint8)

        if results[0].masks is None:
            return segmentation_map

        masks = results[0].masks.data.cpu().numpy()
        for label, mask in enumerate(masks, start=1):
            if label > 255:
                break
            if mask.shape != segmentation_map.shape:
                mask = cv2.resize(
                    mask.astype(np.uint8),
                    (image.width, image.height),
                    interpolation=cv2.INTER_NEAREST,
                )
            segmentation_map[mask.astype(bool)] = label

        return segmentation_map

    def image_to_bgr(self, image: Image) -> np.ndarray:
        row_bytes = image.width * 3
        return (
            np.frombuffer(image.data, dtype=np.uint8)
            .reshape(image.height, image.step)[:, :row_bytes]
            .reshape(image.height, image.width, 3)
            .copy()
        )

    def publish_target_transform(self, transform: TransformStamped) -> None:
        self.target_transform_broadcaster.sendTransform(transform)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ObjectTrackingNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
