#!/usr/bin/env python3

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
from sensor_msgs.msg import Image, PointCloud2, PointField
from tf2_ros import TransformBroadcaster
import ultralytics
from ultralytics.utils import downloads as ultralytics_downloads
from ultralytics import YOLOE


CAPTURE_DIR = Path("/tmp/object_tracking_debug")
SEGMENTED_IMAGE_TOPIC = "/object_tracking/segmented_image"
ORANGE_BGR = np.array([0, 165, 255], dtype=np.uint8)
DEFAULT_OBJECT_TF_NAME = "tracked_object"
MODEL_DIR = Path(__file__).resolve().parents[1] / "models"
MODEL_PATH = MODEL_DIR / "yoloe-26n-seg.pt"


def block_ultralytics_downloads(*args, **kwargs):
    url = kwargs.get("url", args[0] if args else "unknown URL")
    raise RuntimeError(f"Ultralytics attempted to download {url}; install all model assets manually")


def block_ultralytics_asset_download(file, *args, **kwargs):
    file_path = Path(str(file))
    if file_path.is_file():
        return str(file_path)

    local_asset_path = MODEL_DIR / file_path.name
    if local_asset_path.is_file():
        return str(local_asset_path)

    raise FileNotFoundError(
        f"Ultralytics asset '{file_path.name}' is missing. "
        f"Place it at {local_asset_path}; automatic downloads are disabled."
    )


ultralytics_downloads.safe_download = block_ultralytics_downloads
ultralytics_downloads.download = block_ultralytics_downloads
ultralytics_downloads.attempt_download_asset = block_ultralytics_asset_download
ultralytics.download = block_ultralytics_downloads


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
        self.debug = (
            self.declare_parameter("debug", False)
            .get_parameter_value()
            .bool_value
        )
        self.capture = (
            self.declare_parameter("capture", False)
            .get_parameter_value()
            .bool_value
        )
        self.segmentation_confidence = (
            self.declare_parameter("segmentation_confidence", 0.2)
            .get_parameter_value()
            .double_value
        )

        self.latest_image: Optional[Image] = None
        self.latest_pointcloud: Optional[PointCloud2] = None
        self.latest_segmentation_map: Optional[np.ndarray] = None
        self.tracking_active = False
        self.current_object_prompt = ""
        self.current_reference_frame = ""
        self.current_object_tf_name = DEFAULT_OBJECT_TF_NAME
        self.debug_inference_count = 0
        self.capture_frame_count = 0
        self.data_lock = Lock()
        self.model_lock = Lock()
        self.stop_update_loop = Event()
        self.model = YOLOE(MODEL_PATH)

        self.sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sensor_callback_group = ReentrantCallbackGroup()
        self.segmented_image_publisher = (
            self.create_publisher(Image, SEGMENTED_IMAGE_TOPIC, 1)
            if self.debug
            else None
        )

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
        self.get_logger().info(
            f"Segmentation confidence threshold: {self.segmentation_confidence:.2f}"
        )
        self.get_logger().info("Object tracking service ready on object_tracking")
        self.get_logger().info("Target transforms will be broadcast on /tf")
        if self.debug:
            self.get_logger().info(
                f"Debug segmented image will be published on {SEGMENTED_IMAGE_TOPIC}"
            )
        if self.capture:
            self.get_logger().info(f"Capture images will be saved to {CAPTURE_DIR}")

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
        if request.enable:
            print(f"Enabling object tracking for: {request.object_prompt}", flush=True)
            with self.data_lock:
                self.tracking_active = False

            with self.model_lock:
                self.model.set_classes([request.object_prompt])

            with self.data_lock:
                self.current_object_prompt = request.object_prompt
                self.current_reference_frame = request.reference_frame
                self.current_object_tf_name = (
                    request.object_tf_name or DEFAULT_OBJECT_TF_NAME
                )
                self.tracking_active = True
        else:
            print(f"Disabling object tracking", flush=True)
            with self.data_lock:
                self.tracking_active = False

        response.success = True
        response.error = ""
        return response

    def update_pose(self) -> None:
        while rclpy.ok() and not self.stop_update_loop.is_set():
            with self.data_lock:
                tracking_active = self.tracking_active
                image = self.latest_image
                pointcloud = self.latest_pointcloud
                reference_frame = self.current_reference_frame
                object_tf_name = self.current_object_tf_name

            if not tracking_active or image is None or pointcloud is None:
                sleep(0.01)
                continue

            segmentation_map = self.create_segmentation_map(image)
            with self.data_lock:
                self.latest_segmentation_map = segmentation_map

            average_position = self.compute_average_position(segmentation_map, pointcloud)
            if average_position is not None:
                self.publish_target_transform(
                    self.create_target_transform(
                        pointcloud,
                        average_position,
                        reference_frame,
                        object_tf_name,
                    )
                )

            if self.debug or self.capture:
                segmentation_overlay = self.create_segmentation_overlay(image, segmentation_map)
                if self.debug:
                    self.publish_segmented_image(image, segmentation_overlay)
                if self.capture:
                    self.save_capture_images(image, segmentation_overlay, segmentation_map)

    def destroy_node(self) -> bool:
        self.stop_update_loop.set()
        if self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
        return super().destroy_node()

    def create_segmentation_overlay(
        self,
        image: Image,
        segmentation_map: np.ndarray,
    ) -> np.ndarray:
        frame = self.image_to_bgr(image)
        overlay = frame.copy()

        for label in np.unique(segmentation_map):
            if label == 0:
                continue
            mask = segmentation_map == label
            overlay[mask] = (0.5 * frame[mask] + 0.5 * ORANGE_BGR).astype(np.uint8)

        return overlay

    def compute_average_position(
        self,
        segmentation_map: np.ndarray,
        pointcloud: PointCloud2,
    ) -> Optional[np.ndarray]:
        mask = self.resize_segmentation_mask_to_pointcloud(segmentation_map, pointcloud)
        if not np.any(mask):
            return None

        points = self.pointcloud_xyz(pointcloud)
        valid_points = (
            mask
            & np.isfinite(points).all(axis=2)
            & (np.abs(points).sum(axis=2) > np.finfo(np.float32).eps)
        )
        if not np.any(valid_points):
            return None

        return points[valid_points].mean(axis=0)

    def resize_segmentation_mask_to_pointcloud(
        self,
        segmentation_map: np.ndarray,
        pointcloud: PointCloud2,
    ) -> np.ndarray:
        foreground_mask = (segmentation_map > 0).astype(np.uint8)
        return cv2.resize(
            foreground_mask,
            (pointcloud.width, pointcloud.height),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)

    def pointcloud_xyz(self, pointcloud: PointCloud2) -> np.ndarray:
        x_field = self.find_point_field(pointcloud, "x")
        y_field = self.find_point_field(pointcloud, "y")
        z_field = self.find_point_field(pointcloud, "z")
        if (
            x_field is None
            or y_field is None
            or z_field is None
            or x_field.datatype != PointField.FLOAT32
            or y_field.datatype != PointField.FLOAT32
            or z_field.datatype != PointField.FLOAT32
        ):
            raise ValueError("Point cloud must contain FLOAT32 x/y/z fields")

        return np.stack(
            [
                self.pointcloud_float32_field(pointcloud, x_field),
                self.pointcloud_float32_field(pointcloud, y_field),
                self.pointcloud_float32_field(pointcloud, z_field),
            ],
            axis=2,
        )

    def pointcloud_float32_field(self, pointcloud: PointCloud2, field: PointField) -> np.ndarray:
        try:
            data_buffer = memoryview(pointcloud.data)
        except TypeError:
            data_buffer = memoryview(bytes(pointcloud.data))

        required_size = pointcloud.row_step * pointcloud.height
        if data_buffer.nbytes < required_size:
            raise ValueError("Point cloud data is smaller than row_step * height")

        endian = ">" if pointcloud.is_bigendian else "<"
        return np.ndarray(
            shape=(pointcloud.height, pointcloud.width),
            dtype=f"{endian}f4",
            buffer=data_buffer,
            offset=field.offset,
            strides=(pointcloud.row_step, pointcloud.point_step),
        ).astype(np.float32, copy=False)

    def find_point_field(self, pointcloud: PointCloud2, name: str) -> Optional[PointField]:
        for field in pointcloud.fields:
            if field.name == name:
                return field
        return None

    def create_target_transform(
        self,
        pointcloud: PointCloud2,
        position: np.ndarray,
        reference_frame: str,
        object_tf_name: str,
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = pointcloud.header.stamp
        transform.header.frame_id = pointcloud.header.frame_id or reference_frame
        transform.child_frame_id = object_tf_name
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.w = 1.0
        return transform

    def publish_segmented_image(self, image: Image, segmentation_overlay: np.ndarray) -> None:
        if self.segmented_image_publisher is None:
            return

        self.segmented_image_publisher.publish(
            self.bgr_to_image_msg(segmentation_overlay, image)
        )

    def save_capture_images(
        self,
        image: Image,
        segmentation_overlay: np.ndarray,
        segmentation_map: np.ndarray,
    ) -> None:
        frame = self.image_to_bgr(image)
        segmentation_mask = np.where(segmentation_map > 0, 255, 0).astype(np.uint8)
        file_stem = self.next_capture_file_stem(image)

        self.save_bgr_ppm(CAPTURE_DIR / f"{file_stem}_image.ppm", frame)
        self.save_bgr_ppm(
            CAPTURE_DIR / f"{file_stem}_segmented.ppm",
            segmentation_overlay,
        )
        self.save_gray_pgm(CAPTURE_DIR / f"{file_stem}_mask.pgm", segmentation_mask)

    def next_capture_file_stem(self, image: Image) -> str:
        self.capture_frame_count += 1
        stamp = image.header.stamp
        if stamp.sec or stamp.nanosec:
            seconds = stamp.sec
            nanoseconds = stamp.nanosec
        else:
            now = self.get_clock().now().nanoseconds
            seconds, nanoseconds = divmod(now, 1_000_000_000)

        return f"{seconds:010d}_{nanoseconds:09d}_{self.capture_frame_count:06d}"

    def save_bgr_ppm(self, path: Path, image: np.ndarray) -> None:
        height, width = image.shape[:2]
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("wb") as image_file:
            image_file.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
            image_file.write(image[:, :, ::-1].tobytes())

    def save_gray_pgm(self, path: Path, image: np.ndarray) -> None:
        height, width = image.shape[:2]
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("wb") as image_file:
            image_file.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
            image_file.write(image.tobytes())

    def bgr_to_image_msg(self, image: np.ndarray, source_image: Image) -> Image:
        msg = Image()
        msg.header = source_image.header
        msg.height, msg.width = image.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = image.tobytes()
        return msg

    def create_segmentation_map(self, image: Image) -> np.ndarray:
        frame = self.image_to_bgr(image)
        with self.model_lock:
            results = self.model(
                frame,
                conf=self.segmentation_confidence,
                verbose=False,
            )
        segmentation_map = np.zeros((image.height, image.width), dtype=np.uint8)

        if results[0].masks is None:
            self.log_segmentation_debug(image, 0, 0)
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

        self.log_segmentation_debug(image, len(masks), int(np.count_nonzero(segmentation_map)))
        return segmentation_map

    def log_segmentation_debug(
        self,
        image: Image,
        mask_count: int,
        foreground_pixels: int,
    ) -> None:
        if not self.debug:
            return

        self.debug_inference_count += 1
        if self.debug_inference_count % 10 != 1:
            return

        self.get_logger().info(
            f"YOLOE prompt '{self.current_object_prompt}': "
            f"{mask_count} masks, {foreground_pixels} foreground pixels "
            f"on {image.width}x{image.height}"
        )

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
