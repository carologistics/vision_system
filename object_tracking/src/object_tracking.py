#!/usr/bin/env python3

import os
from pathlib import Path
from threading import Event, Lock, Thread
from time import monotonic, sleep
from typing import Optional

os.environ["YOLO_AUTOINSTALL"] = "false"
os.environ["YOLO_OFFLINE"] = "true"

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from robotino_vision_msgs.action import AcquireObjectTracking
from robotino_vision_msgs.srv import ToggleObjectTracking
from sensor_msgs.msg import Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
import ultralytics
from ultralytics.utils import downloads as ultralytics_downloads
from ultralytics import YOLOE


CAPTURE_DIR = Path("/tmp/object_tracking_debug")
SEGMENTED_IMAGE_TOPIC = "/object_tracking/segmented_image"
ORANGE_BGR = np.array([0, 165, 255], dtype=np.uint8)
DEFAULT_OBJECT_TF_NAME = "tracked_object"
MODEL_DIR = Path(__file__).resolve().parents[1] / "models"
MODEL_PATH = MODEL_DIR / "yoloe-26n-seg.pt"

# OpenCV HSV uses hue in [0, 179]. Saturation/value lower bounds keep low-color
# grey or shadowed pixels from matching arbitrary hues.
COLOR_HSV_INTERVALS = {
    "blue": (((95, 50, 40), (130, 255, 255)),),
    "green": (((40, 45, 35), (85, 255, 255)),),
    "red": (((0, 60, 35), (10, 255, 255)), ((170, 60, 35), (179, 255, 255))),
    "yellow": (((18, 55, 45), (40, 255, 255)),),
}
VALID_TARGET_COLORS = tuple(sorted(COLOR_HSV_INTERVALS))


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
        self.segmentation_height_fraction = (
            self.declare_parameter("segmentation_height_fraction", 0.65)
            .get_parameter_value()
            .double_value
        )
        if not 0.0 <= self.segmentation_height_fraction <= 1.0:
            raise ValueError(
                "segmentation_height_fraction must be in [0.0, 1.0] "
                f"but is {self.segmentation_height_fraction}"
            )
        self.approach_distance = (
            self.declare_parameter("approach_distance", 0.35)
            .get_parameter_value()
            .double_value
        )
        if self.approach_distance < 0.0:
            raise ValueError(
                "approach_distance must be >= 0.0 "
                f"but is {self.approach_distance}"
            )
        self.approach_frame_suffix = (
            self.declare_parameter("approach_frame_suffix", "_approach")
            .get_parameter_value()
            .string_value
        )
        if not self.approach_frame_suffix:
            raise ValueError("approach_frame_suffix must not be empty")
        self.base_frame = self.namespaced_frame("base_link")
        self.odom_frame = self.namespaced_frame("odom")
        self.target_parent_frame = self.namespaced_frame("gripper_cam")

        self.latest_image: Optional[Image] = None
        self.latest_pointcloud: Optional[PointCloud2] = None
        self.latest_segmentation_map: Optional[np.ndarray] = None
        self.tracking_active = False
        self.current_object_prompt = ""
        self.current_target_color = ""
        self.current_reference_frame = self.base_frame
        self.current_distance_threshold = 10.0
        self.current_approach_distance = self.approach_distance
        self.current_segmentation_confidence = self.segmentation_confidence
        self.current_object_tf_name = DEFAULT_OBJECT_TF_NAME
        self.acquire_session = None
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
        self.acquire_object_tracking_action = ActionServer(
            self,
            AcquireObjectTracking,
            "acquire_object_tracking",
            execute_callback=self.execute_acquire_object_tracking,
            cancel_callback=self.cancel_acquire_object_tracking,
            callback_group=self.sensor_callback_group,
        )
        self.target_transform_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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
        self.get_logger().info(
            "Segmentation height ROI: "
            f"top {self.segmentation_height_fraction * 100.0:.1f}% of the image"
        )
        self.get_logger().info(
            f"Target parent TF frame: {self.target_parent_frame}"
        )
        self.get_logger().info(
            "Object approach TF: "
            f"parent={self.base_frame} suffix={self.approach_frame_suffix} "
            f"distance={self.approach_distance:.3f} m"
        )
        self.get_logger().info(f"Acquired object TFs use parent frame {self.odom_frame}")
        self.get_logger().info("Object tracking service ready on object_tracking")
        self.get_logger().info("Object acquisition action ready on acquire_object_tracking")
        self.get_logger().info("Target transforms will be broadcast on /tf")
        if self.debug:
            self.get_logger().info(
                f"Debug segmented image will be published on {SEGMENTED_IMAGE_TOPIC}"
            )
        if self.capture:
            self.get_logger().info(f"Capture images will be saved to {CAPTURE_DIR}")

    def namespaced_frame(self, frame_name: str) -> str:
        frame_name = frame_name.strip("/")
        namespace = self.get_namespace().strip("/")
        if namespace and not (
            frame_name == namespace or frame_name.startswith(f"{namespace}/")
        ):
            return f"{namespace}/{frame_name}"
        return frame_name

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
            requested_confidence = request.segmentation_confidence
            if requested_confidence < 0.0 or requested_confidence > 1.0:
                response.success = False
                response.error = "segmentation_confidence must be in the range (0.0, 1.0]"
                return response

            requested_approach_distance = request.approach_distance
            if requested_approach_distance < 0.0:
                response.success = False
                response.error = "approach_distance must be >= 0.0"
                return response
            approach_distance = (
                requested_approach_distance
                if requested_approach_distance > 0.0
                else self.approach_distance
            )

            requested_target_color = request.target_color.strip().lower()
            if requested_target_color and requested_target_color not in COLOR_HSV_INTERVALS:
                response.success = False
                response.error = (
                    "target_color must be empty or one of: "
                    + ", ".join(VALID_TARGET_COLORS)
                )
                return response

            target_color_msg = (
                f", target_color {requested_target_color}"
                if requested_target_color
                else ""
            )
            print(
                f"Enabling object tracking for: {request.object_prompt} "
                f"with confidence {requested_confidence:.2f}, "
                f"approach distance {approach_distance:.3f} m{target_color_msg}",
                flush=True,
            )
            with self.data_lock:
                self.tracking_active = False

            with self.model_lock:
                self.model.set_classes([request.object_prompt])

            with self.data_lock:
                self.current_object_prompt = request.object_prompt
                self.current_target_color = requested_target_color
                reference_frame = request.reference_frame or "base_link"
                self.current_reference_frame = reference_frame
                self.current_distance_threshold = request.distance_threshold
                self.current_approach_distance = approach_distance
                self.current_segmentation_confidence = requested_confidence
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
        print(1)
        return response

    def cancel_acquire_object_tracking(self, goal_handle) -> CancelResponse:
        self.get_logger().warn("Cancel requested for object acquisition")
        return CancelResponse.ACCEPT

    def execute_acquire_object_tracking(self, goal_handle):
        request = goal_handle.request
        result = AcquireObjectTracking.Result()
        error = self.validate_acquire_request(request)
        if error:
            result.success = False
            result.error = error
            result.object_tf_name = ""
            result.approach_tf_name = ""
            self.get_logger().error(f"Rejecting object acquisition: {error}")
            goal_handle.abort()
            return result

        object_prompt = request.object_prompt.strip()
        target_color = request.target_color.strip().lower()
        object_tf_name = request.object_tf_name.strip()
        approach_tf_name = request.approach_tf_name.strip()
        reference_frame = self.namespaced_frame(request.reference_frame)
        object_tf_frame = self.namespaced_frame(object_tf_name)
        approach_tf_frame = self.namespaced_frame(approach_tf_name)

        with self.data_lock:
            if self.acquire_session is not None:
                result.success = False
                result.error = "another object acquisition is already active"
                result.object_tf_name = object_tf_frame
                result.approach_tf_name = approach_tf_frame
                self.get_logger().error(result.error)
                goal_handle.abort()
                return result
            self.tracking_active = False

        with self.model_lock:
            self.model.set_classes([object_prompt])

        done_event = Event()
        session = {
            "goal_handle": goal_handle,
            "event": done_event,
            "object_prompt": object_prompt,
            "target_color": target_color,
            "reference_frame": reference_frame,
            "distance_threshold": request.distance_threshold,
            "approach_distance": request.approach_distance,
            "segmentation_confidence": request.segmentation_confidence,
            "object_tf_name": object_tf_name,
            "approach_tf_name": approach_tf_name,
            "object_tf_frame": object_tf_frame,
            "approach_tf_frame": approach_tf_frame,
            "min_stable_frames": request.min_stable_frames,
            "max_position_jump": request.max_position_jump,
            "stable_frames": 0,
            "last_object_position": None,
            "last_object_distance": 0.0,
            "last_debug_log_time": 0.0,
            "last_debug_message": "",
            "success": False,
            "error": "",
        }

        with self.data_lock:
            self.current_object_prompt = object_prompt
            self.current_target_color = target_color
            self.acquire_session = session

        self.get_logger().info(
            "Acquiring object: "
            f"prompt='{object_prompt}' reference_frame={reference_frame} "
            f"distance_threshold={request.distance_threshold:.3f} "
            f"approach_distance={request.approach_distance:.3f} "
            f"segmentation_confidence={request.segmentation_confidence:.3f} "
            f"target_color='{target_color}' timeout={request.timeout_sec:.3f}s "
            f"min_stable_frames={request.min_stable_frames} "
            f"max_position_jump={request.max_position_jump:.3f} "
            f"object_tf={object_tf_frame} approach_tf={approach_tf_frame}"
        )

        start_time = monotonic()
        while not done_event.wait(0.05):
            if goal_handle.is_cancel_requested:
                self.clear_acquire_session(session)
                result.success = False
                result.error = "object acquisition canceled"
                result.object_tf_name = object_tf_frame
                result.approach_tf_name = approach_tf_frame
                goal_handle.canceled()
                self.get_logger().warn(result.error)
                return result

            if monotonic() - start_time >= request.timeout_sec:
                self.clear_acquire_session(session)
                result.success = False
                result.error = (
                    "object acquisition timed out after "
                    f"{request.timeout_sec:.3f}s"
                )
                result.object_tf_name = object_tf_frame
                result.approach_tf_name = approach_tf_frame
                goal_handle.abort()
                self.get_logger().error(result.error)
                return result

        with self.data_lock:
            success = session["success"]
            error = session["error"]
            if self.acquire_session is session:
                self.acquire_session = None

        result.success = bool(success)
        result.error = error
        result.object_tf_name = object_tf_frame
        result.approach_tf_name = approach_tf_frame

        if result.success:
            goal_handle.succeed()
            self.get_logger().info(
                "Object acquisition succeeded: "
                f"object_tf={result.object_tf_name} approach_tf={result.approach_tf_name}"
            )
        else:
            goal_handle.abort()
            self.get_logger().error(f"Object acquisition failed: {result.error}")
        return result

    def validate_acquire_request(self, request: AcquireObjectTracking.Goal) -> str:
        if not request.object_prompt.strip():
            return "object_prompt must not be empty"
        if not request.reference_frame.strip():
            return "reference_frame must not be empty"
        if request.distance_threshold <= 0.0:
            return "distance_threshold must be > 0.0"
        if request.approach_distance < 0.0:
            return "approach_distance must be >= 0.0"
        if not 0.0 <= request.segmentation_confidence <= 1.0:
            return "segmentation_confidence must be in [0.0, 1.0]"
        target_color = request.target_color.strip().lower()
        if target_color and target_color not in COLOR_HSV_INTERVALS:
            return "target_color must be empty or one of: " + ", ".join(VALID_TARGET_COLORS)
        if not request.object_tf_name.strip():
            return "object_tf_name must not be empty"
        if not request.approach_tf_name.strip():
            return "approach_tf_name must not be empty"
        if request.object_tf_name.strip() == request.approach_tf_name.strip():
            return "object_tf_name and approach_tf_name must be different"
        if request.timeout_sec <= 0.0:
            return "timeout_sec must be > 0.0"
        if request.min_stable_frames == 0:
            return "min_stable_frames must be > 0"
        if request.max_position_jump < 0.0:
            return "max_position_jump must be >= 0.0"
        return ""

    def clear_acquire_session(self, session) -> None:
        with self.data_lock:
            if self.acquire_session is session:
                self.acquire_session = None

    def update_pose(self) -> None:
        while rclpy.ok() and not self.stop_update_loop.is_set():
            with self.data_lock:
                tracking_active = self.tracking_active
                acquire_session = self.acquire_session
                image = self.latest_image
                pointcloud = self.latest_pointcloud
                if acquire_session is not None:
                    reference_frame = acquire_session["reference_frame"]
                    distance_threshold = acquire_session["distance_threshold"]
                    approach_distance = acquire_session["approach_distance"]
                    segmentation_confidence = acquire_session["segmentation_confidence"]
                    target_color = acquire_session["target_color"]
                    object_tf_name = acquire_session["object_tf_name"]
                else:
                    reference_frame = self.current_reference_frame
                    distance_threshold = self.current_distance_threshold
                    approach_distance = self.current_approach_distance
                    segmentation_confidence = self.current_segmentation_confidence
                    target_color = self.current_target_color
                    object_tf_name = self.current_object_tf_name

            if (not tracking_active and acquire_session is None) or image is None or pointcloud is None:
                if acquire_session is not None:
                    missing = []
                    if image is None:
                        missing.append(f"image on {self.image_topic}")
                    if pointcloud is None:
                        missing.append(f"pointcloud on {self.pointcloud_topic}")
                    self.log_acquire_info(
                        acquire_session,
                        "waiting for " + " and ".join(missing),
                    )
                sleep(0.01)
                continue

            segmentation_map = self.create_segmentation_map(
                image, segmentation_confidence, target_color
            )
            segmentation_map = self.apply_segmentation_height_cut(segmentation_map)
            with self.data_lock:
                self.latest_segmentation_map = segmentation_map

            candidate_positions = self.compute_candidate_positions(
                segmentation_map, pointcloud
            )
            selected_candidate, rejection_reason = self.select_candidate_position(
                candidate_positions,
                reference_frame,
                pointcloud,
                distance_threshold,
            )

            if selected_candidate is not None:
                object_position, object_distance = selected_candidate
                if tracking_active:
                    self.publish_target_transform(
                        self.create_target_transform(
                            pointcloud,
                            object_position,
                            object_tf_name,
                        )
                    )
                    self.publish_target_transform(
                        self.create_approach_transform(
                            pointcloud,
                            object_position,
                            object_tf_name,
                            approach_distance,
                        )
                    )
                if acquire_session is not None:
                    self.update_acquire_session(
                        acquire_session,
                        pointcloud,
                        object_position,
                        object_distance,
                    )
            elif acquire_session is not None:
                self.log_acquire_info(acquire_session, rejection_reason)
                self.reset_acquire_stability(acquire_session)
                self.publish_acquire_feedback(acquire_session, False, 0, 0.0)

            if self.debug or self.capture:
                segmentation_overlay = self.create_segmentation_overlay(image, segmentation_map)
                if self.debug:
                    self.publish_segmented_image(image, segmentation_overlay)
                if self.capture:
                    self.save_capture_images(image, segmentation_overlay, segmentation_map)

    def select_candidate_position(
        self,
        candidate_positions: list[np.ndarray],
        reference_frame: str,
        pointcloud: PointCloud2,
        distance_threshold: float,
    ) -> tuple[Optional[tuple[np.ndarray, float]], str]:
        if not candidate_positions:
            return None, "no usable detections from segmentation and pointcloud"

        reference_position = self.reference_position(
            reference_frame,
            pointcloud.header.stamp,
        )
        candidate_distances = [
            float(np.linalg.norm(position - reference_position))
            for position in candidate_positions
        ]
        closest_index = int(np.argmin(candidate_distances))
        closest_distance = candidate_distances[closest_index]
        closest_position = candidate_positions[closest_index]
        xy_distance = float(np.linalg.norm(closest_position[:2] - reference_position[:2]))
        z_distance = float(abs(closest_position[2] - reference_position[2]))
        if closest_distance > distance_threshold:
            return None, (
                "closest detection too far: "
                f"distance={closest_distance:.3f} m allowed={distance_threshold:.3f} m "
                f"xy={xy_distance:.3f} m z_delta={z_distance:.3f} m "
                f"candidates={len(candidate_positions)}"
            )
        return (closest_position, closest_distance), ""

    def log_acquire_info(self, session, message: str, interval_sec: float = 1.0) -> None:
        now = monotonic()
        last_time = session.get("last_debug_log_time", 0.0)
        last_message = session.get("last_debug_message", "")
        if message == last_message and now - last_time < interval_sec:
            return

        session["last_debug_log_time"] = now
        session["last_debug_message"] = message
        self.get_logger().info(f"Object acquisition: {message}")

    def reset_acquire_stability(self, session) -> None:
        with self.data_lock:
            if self.acquire_session is session:
                session["stable_frames"] = 0
                session["last_object_position"] = None
                session["last_object_distance"] = 0.0

    def update_acquire_session(
        self,
        session,
        pointcloud: PointCloud2,
        object_position: np.ndarray,
        object_distance: float,
    ) -> None:
        try:
            acquired_transforms, object_in_odom = self.create_acquired_target_transforms(
                pointcloud,
                object_position,
                session["object_tf_name"],
                session["approach_tf_name"],
                session["approach_distance"],
            )
        except Exception as exc:
            self.get_logger().warn(f"Could not create object acquisition TFs: {exc}")
            self.reset_acquire_stability(session)
            self.publish_acquire_feedback(session, False, 0, object_distance)
            return

        with self.data_lock:
            if self.acquire_session is not session:
                return

            last_position = session["last_object_position"]
            if last_position is None:
                stable_frames = 1
            else:
                position_jump = float(np.linalg.norm(object_in_odom - last_position))
                stable_frames = (
                    session["stable_frames"] + 1
                    if position_jump <= session["max_position_jump"]
                    else 1
                )

            session["stable_frames"] = stable_frames
            session["last_object_position"] = object_in_odom
            session["last_object_distance"] = object_distance
            acquired = stable_frames >= session["min_stable_frames"]
            if acquired:
                session["success"] = True
                session["error"] = ""

        for transform in acquired_transforms:
            self.publish_target_transform(transform)

        if acquired:
            session["event"].set()

        self.log_acquire_info(
            session,
            "valid detection: "
            f"distance={object_distance:.3f} m "
            f"stable_frames={stable_frames}/{session['min_stable_frames']}",
        )
        self.publish_acquire_feedback(
            session,
            True,
            stable_frames,
            object_distance,
        )

    def publish_acquire_feedback(
        self,
        session,
        detection_valid: bool,
        stable_frames: int,
        object_distance: float,
    ) -> None:
        feedback = AcquireObjectTracking.Feedback()
        feedback.detection_valid = detection_valid
        feedback.stable_frames = stable_frames
        feedback.object_distance = object_distance
        try:
            session["goal_handle"].publish_feedback(feedback)
        except Exception as exc:
            self.get_logger().debug(f"Could not publish object acquisition feedback: {exc}")

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

    def apply_segmentation_height_cut(self, segmentation_map: np.ndarray) -> np.ndarray:
        cutoff_row = int(
            np.ceil(segmentation_map.shape[0] * self.segmentation_height_fraction)
        )
        filtered_map = segmentation_map.copy()
        filtered_map[cutoff_row:, :] = 0
        return filtered_map

    def compute_candidate_positions(
        self,
        segmentation_map: np.ndarray,
        pointcloud: PointCloud2,
    ) -> list[np.ndarray]:
        points = self.pointcloud_xyz(pointcloud)
        candidate_positions = []

        for label in np.unique(segmentation_map):
            if label == 0:
                continue

            mask = self.resize_segmentation_mask_to_pointcloud(
                segmentation_map == label, pointcloud
            )
            valid_points = (
                mask
                & np.isfinite(points).all(axis=2)
                & (np.abs(points).sum(axis=2) > np.finfo(np.float32).eps)
            )
            if np.any(valid_points):
                candidate_positions.append(points[valid_points].mean(axis=0))

        return candidate_positions

    def resize_segmentation_mask_to_pointcloud(
        self,
        segmentation_mask: np.ndarray,
        pointcloud: PointCloud2,
    ) -> np.ndarray:
        return cv2.resize(
            segmentation_mask.astype(np.uint8),
            (pointcloud.width, pointcloud.height),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)

    def reference_position(self, reference_frame: str, stamp) -> np.ndarray:
        reference_transform = self.tf_buffer.lookup_transform(
            self.target_parent_frame,
            reference_frame,
            Time.from_msg(stamp),
        )
        translation = reference_transform.transform.translation
        return np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float32,
        )

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
        object_tf_name: str,
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = pointcloud.header.stamp
        transform.header.frame_id = self.target_parent_frame
        transform.child_frame_id = self.namespaced_frame(object_tf_name)
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.w = 1.0
        return transform

    def create_approach_transform(
        self,
        pointcloud: PointCloud2,
        object_position: np.ndarray,
        object_tf_name: str,
        approach_distance: float,
    ) -> Optional[TransformStamped]:
        try:
            object_in_base = self.transform_position(
                object_position,
                self.target_parent_frame,
                self.base_frame,
                pointcloud.header.stamp,
            )
        except Exception as exc:
            self.get_logger().warn(
                "Could not create approach TF from "
                f"{self.base_frame} to {object_tf_name}: {exc}"
            )
            return None

        object_xy = object_in_base[:2].astype(np.float64, copy=False)
        object_distance = float(np.linalg.norm(object_xy))
        if object_distance <= np.finfo(np.float64).eps:
            approach_xy = object_xy
        else:
            target_distance = max(object_distance - approach_distance, 0.0)
            approach_xy = object_xy * (target_distance / object_distance)

        transform = TransformStamped()
        transform.header.stamp = pointcloud.header.stamp
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.namespaced_frame(
            f"{object_tf_name}{self.approach_frame_suffix}"
        )
        yaw_to_object = 0.0
        if object_distance > np.finfo(np.float64).eps:
            yaw_to_object = float(np.arctan2(object_xy[1], object_xy[0]))

        transform.transform.translation.x = float(approach_xy[0])
        transform.transform.translation.y = float(approach_xy[1])
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = float(np.sin(0.5 * yaw_to_object))
        transform.transform.rotation.w = float(np.cos(0.5 * yaw_to_object))
        return transform

    def create_acquired_target_transforms(
        self,
        pointcloud: PointCloud2,
        object_position: np.ndarray,
        object_tf_name: str,
        approach_tf_name: str,
        approach_distance: float,
    ) -> tuple[list[TransformStamped], np.ndarray]:
        stamp = pointcloud.header.stamp
        object_in_odom = self.transform_position(
            object_position,
            self.target_parent_frame,
            self.odom_frame,
            stamp,
        )
        base_in_odom = self.transform_position(
            np.zeros(3, dtype=np.float64),
            self.base_frame,
            self.odom_frame,
            stamp,
        )

        object_transform = TransformStamped()
        object_transform.header.stamp = stamp
        object_transform.header.frame_id = self.odom_frame
        object_transform.child_frame_id = self.namespaced_frame(object_tf_name)
        object_transform.transform.translation.x = float(object_in_odom[0])
        object_transform.transform.translation.y = float(object_in_odom[1])
        object_transform.transform.translation.z = float(object_in_odom[2])
        object_transform.transform.rotation.w = 1.0

        base_xy = base_in_odom[:2].astype(np.float64, copy=False)
        object_xy = object_in_odom[:2].astype(np.float64, copy=False)
        base_to_object = object_xy - base_xy
        object_distance = float(np.linalg.norm(base_to_object))
        if object_distance <= np.finfo(np.float64).eps:
            approach_xy = base_xy
            yaw_to_object = 0.0
        else:
            approach_xy = object_xy - approach_distance * base_to_object / object_distance
            yaw_to_object = float(np.arctan2(base_to_object[1], base_to_object[0]))

        approach_transform = TransformStamped()
        approach_transform.header.stamp = stamp
        approach_transform.header.frame_id = self.odom_frame
        approach_transform.child_frame_id = self.namespaced_frame(approach_tf_name)
        approach_transform.transform.translation.x = float(approach_xy[0])
        approach_transform.transform.translation.y = float(approach_xy[1])
        approach_transform.transform.translation.z = 0.0
        approach_transform.transform.rotation.z = float(np.sin(0.5 * yaw_to_object))
        approach_transform.transform.rotation.w = float(np.cos(0.5 * yaw_to_object))

        return [object_transform, approach_transform], object_in_odom

    def transform_position(
        self,
        position: np.ndarray,
        source_frame: str,
        target_frame: str,
        stamp,
    ) -> np.ndarray:
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time.from_msg(stamp),
        )
        rotation = self.quaternion_to_rotation_matrix(transform.transform.rotation)
        translation = transform.transform.translation
        return rotation @ position.astype(np.float64, copy=False) + np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float64,
        )

    def quaternion_to_rotation_matrix(self, quaternion) -> np.ndarray:
        x = float(quaternion.x)
        y = float(quaternion.y)
        z = float(quaternion.z)
        w = float(quaternion.w)
        norm = np.sqrt(x * x + y * y + z * z + w * w)
        if norm <= np.finfo(np.float64).eps:
            return np.eye(3, dtype=np.float64)

        x /= norm
        y /= norm
        z /= norm
        w /= norm
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

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

    def create_segmentation_map(
        self,
        image: Image,
        segmentation_confidence: float,
        target_color: str,
    ) -> np.ndarray:
        frame = self.image_to_bgr(image)
        with self.model_lock:
            results = self.model(
                frame,
                conf=segmentation_confidence,
                verbose=False,
            )
        segmentation_map = np.zeros((image.height, image.width), dtype=np.uint8)

        if results[0].masks is None:
            self.log_segmentation_debug(image, 0, 0, 0, target_color)
            return segmentation_map

        masks = results[0].masks.data.cpu().numpy()
        next_label = 1
        for mask in masks:
            if next_label > 255:
                break
            if mask.shape != segmentation_map.shape:
                mask = cv2.resize(
                    mask.astype(np.uint8),
                    (image.width, image.height),
                    interpolation=cv2.INTER_NEAREST,
                )
            mask_bool = mask.astype(bool)
            if target_color and not self.mask_matches_target_color(
                frame, mask_bool, target_color
            ):
                continue
            segmentation_map[mask_bool] = next_label
            next_label += 1

        accepted_mask_count = next_label - 1
        self.log_segmentation_debug(
            image,
            len(masks),
            accepted_mask_count,
            int(np.count_nonzero(segmentation_map)),
            target_color,
        )
        return segmentation_map

    def mask_matches_target_color(
        self,
        frame_bgr: np.ndarray,
        mask: np.ndarray,
        target_color: str,
    ) -> bool:
        pixels_bgr = frame_bgr[mask]
        if pixels_bgr.size == 0:
            return False

        median_bgr = np.median(pixels_bgr, axis=0).astype(np.uint8).reshape(1, 1, 3)
        median_hsv = cv2.cvtColor(median_bgr, cv2.COLOR_BGR2HSV)[0, 0]

        return any(
            self.hsv_in_interval(median_hsv, lower, upper)
            for lower, upper in COLOR_HSV_INTERVALS[target_color]
        )

    @staticmethod
    def hsv_in_interval(
        hsv: np.ndarray,
        lower: tuple[int, int, int],
        upper: tuple[int, int, int],
    ) -> bool:
        return all(
            int(low) <= int(value) <= int(high)
            for value, low, high in zip(hsv, lower, upper)
        )

    def log_segmentation_debug(
        self,
        image: Image,
        mask_count: int,
        accepted_mask_count: int,
        foreground_pixels: int,
        target_color: str,
    ) -> None:
        if not self.debug:
            return

        self.debug_inference_count += 1
        if self.debug_inference_count % 10 != 1:
            return

        target_color_msg = f", target color '{target_color}'" if target_color else ""
        self.get_logger().info(
            f"YOLOE prompt '{self.current_object_prompt}': "
            f"{mask_count} masks, {accepted_mask_count} accepted"
            f"{target_color_msg}, {foreground_pixels} foreground pixels "
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
