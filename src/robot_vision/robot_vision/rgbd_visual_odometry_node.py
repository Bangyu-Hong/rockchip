#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import List, Optional, Sequence, Tuple

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


DEFAULT_COLOR_TOPIC = '/camera/color/image_raw'
DEFAULT_DEPTH_TOPIC = '/camera/depth/image_raw'
DEFAULT_CAMERA_INFO_TOPIC = '/camera/color/camera_info'
DEFAULT_RGBD_ODOM_TOPIC = '/rgbd/odom'
DEFAULT_ODOM_FRAME_ID = 'rgbd_odom'
DEFAULT_CHILD_FRAME_ID = 'camera_color_optical_frame'
DEFAULT_MIN_DEPTH_M = 0.2
DEFAULT_MAX_DEPTH_M = 2.5
DEFAULT_DEPTH_SCALE = 1000.0
DEFAULT_FLIP_DEPTH_HORIZONTAL = True
DEFAULT_ENABLE_DEPTH_PADDING = True

# Runtime debug switch. The node still publishes /rgbd/odom when this is off.
DEFAULT_PUBLISH_DEBUG_IMAGE = False


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_rotation_matrix(rotation: np.ndarray) -> Quaternion:
    trace = float(np.trace(rotation))
    q = Quaternion()
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        q.w = 0.25 * scale
        q.x = (rotation[2, 1] - rotation[1, 2]) / scale
        q.y = (rotation[0, 2] - rotation[2, 0]) / scale
        q.z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        q.w = (rotation[2, 1] - rotation[1, 2]) / scale
        q.x = 0.25 * scale
        q.y = (rotation[0, 1] + rotation[1, 0]) / scale
        q.z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        q.w = (rotation[0, 2] - rotation[2, 0]) / scale
        q.x = (rotation[0, 1] + rotation[1, 0]) / scale
        q.y = 0.25 * scale
        q.z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        q.w = (rotation[1, 0] - rotation[0, 1]) / scale
        q.x = (rotation[0, 2] + rotation[2, 0]) / scale
        q.y = (rotation[1, 2] + rotation[2, 1]) / scale
        q.z = 0.25 * scale
    return q


def pad_depth_to_color_size(depth_img: np.ndarray, target_h: int, target_w: int) -> np.ndarray:
    depth_h, depth_w = depth_img.shape[:2]
    if depth_h == target_h and depth_w == target_w:
        return depth_img
    pad_top = max(0, (target_h - depth_h) // 2)
    pad_bottom = max(0, target_h - depth_h - pad_top)
    pad_left = max(0, (target_w - depth_w) // 2)
    pad_right = max(0, target_w - depth_w - pad_left)
    padded = cv2.copyMakeBorder(
        depth_img,
        pad_top,
        pad_bottom,
        pad_left,
        pad_right,
        cv2.BORDER_CONSTANT,
        value=0,
    )
    return padded[:target_h, :target_w]


def depth_to_meters(depth_values: np.ndarray, depth_scale: float) -> np.ndarray:
    values = depth_values.astype(np.float32)
    if np.issubdtype(depth_values.dtype, np.floating):
        return values
    return values / float(depth_scale)


def sample_depth_at(
    depth_img: np.ndarray,
    x: float,
    y: float,
    radius: int,
    depth_scale: float,
    min_depth_m: float,
    max_depth_m: float,
) -> Optional[float]:
    height, width = depth_img.shape[:2]
    px = int(round(x))
    py = int(round(y))
    left = max(0, px - radius)
    right = min(width, px + radius + 1)
    top = max(0, py - radius)
    bottom = min(height, py + radius + 1)
    if left >= right or top >= bottom:
        return None
    patch = depth_img[top:bottom, left:right]
    valid = patch[patch > 0]
    if valid.size == 0:
        return None
    meters = depth_to_meters(valid, depth_scale)
    meters = meters[(meters >= min_depth_m) & (meters <= max_depth_m)]
    if meters.size == 0:
        return None
    return float(np.median(meters))


def pixel_to_camera_xyz(
    x: float,
    y: float,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    return np.array([
        (x - cx) * depth_m / fx,
        (y - cy) * depth_m / fy,
        depth_m,
    ], dtype=np.float32)


def build_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation.reshape(3)
    return transform


class RgbdVisualOdometryNode(Node):
    def __init__(self):
        super().__init__('rgbd_visual_odometry_node')

        self.declare_parameter('color_topic', DEFAULT_COLOR_TOPIC)
        self.declare_parameter('depth_topic', DEFAULT_DEPTH_TOPIC)
        self.declare_parameter('camera_info_topic', DEFAULT_CAMERA_INFO_TOPIC)
        self.declare_parameter('odom_topic', DEFAULT_RGBD_ODOM_TOPIC)
        self.declare_parameter('odom_frame_id', DEFAULT_ODOM_FRAME_ID)
        self.declare_parameter('child_frame_id', DEFAULT_CHILD_FRAME_ID)
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('debug_image_topic', '/rgbd_odom/debug/features')
        self.declare_parameter('publish_debug_image', DEFAULT_PUBLISH_DEBUG_IMAGE)
        self.declare_parameter('min_depth_m', DEFAULT_MIN_DEPTH_M)
        self.declare_parameter('max_depth_m', DEFAULT_MAX_DEPTH_M)
        self.declare_parameter('depth_scale', DEFAULT_DEPTH_SCALE)
        self.declare_parameter('flip_depth_horizontal', DEFAULT_FLIP_DEPTH_HORIZONTAL)
        self.declare_parameter('enable_depth_padding', DEFAULT_ENABLE_DEPTH_PADDING)
        self.declare_parameter('depth_sample_radius', 2)
        self.declare_parameter('max_features', 800)
        self.declare_parameter('match_ratio', 0.75)
        self.declare_parameter('min_matches', 30)
        self.declare_parameter('min_inliers', 20)
        self.declare_parameter('pnp_reprojection_error', 4.0)
        self.declare_parameter('max_translation_per_frame_m', 0.35)
        self.declare_parameter('max_rotation_per_frame_rad', 0.65)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop_sec', 0.10)

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        self.min_depth_m = self.get_parameter('min_depth_m').value
        self.max_depth_m = self.get_parameter('max_depth_m').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.flip_depth_horizontal = self.get_parameter('flip_depth_horizontal').value
        self.enable_depth_padding = self.get_parameter('enable_depth_padding').value
        self.depth_sample_radius = int(self.get_parameter('depth_sample_radius').value)
        self.match_ratio = float(self.get_parameter('match_ratio').value)
        self.min_matches = int(self.get_parameter('min_matches').value)
        self.min_inliers = int(self.get_parameter('min_inliers').value)
        self.pnp_reprojection_error = float(
            self.get_parameter('pnp_reprojection_error').value
        )
        self.max_translation_per_frame_m = float(
            self.get_parameter('max_translation_per_frame_m').value
        )
        self.max_rotation_per_frame_rad = float(
            self.get_parameter('max_rotation_per_frame_rad').value
        )

        max_features = int(self.get_parameter('max_features').value)
        self.detector = cv2.ORB_create(nfeatures=max_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.bridge = CvBridge()
        self.pose = np.eye(4, dtype=np.float64)
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.prev_points_3d: List[Optional[np.ndarray]] = []
        self.fx = self.fy = self.cx = self.cy = None
        self.frame_count = 0
        self.accepted_count = 0

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 5)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        color_sub = message_filters.Subscriber(self, Image, self.color_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            queue_size=int(self.get_parameter('sync_queue_size').value),
            slop=float(self.get_parameter('sync_slop_sec').value),
        )
        self.sync.registerCallback(self.rgbd_callback)

        self.get_logger().info(
            'RGB-D视觉里程计已启动: color=%s depth=%s info=%s -> %s, '
            'depth_range=%.2f-%.2fm, publish_tf=%s'
            % (
                self.color_topic,
                self.depth_topic,
                self.camera_info_topic,
                self.odom_topic,
                self.min_depth_m,
                self.max_depth_m,
                self.publish_tf,
            )
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.get_logger().info(
            'RGB-D视觉里程计内参就绪: fx=%.2f fy=%.2f cx=%.2f cy=%.2f'
            % (self.fx, self.fy, self.cx, self.cy),
            throttle_duration_sec=5.0,
        )

    def rgbd_callback(self, color_msg: Image, depth_msg: Image):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            self.get_logger().warn(
                f'等待相机内参: {self.camera_info_topic}',
                throttle_duration_sec=2.0,
            )
            return

        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'RGB-D图像转换失败: {exc}', throttle_duration_sec=2.0)
            return

        if self.flip_depth_horizontal:
            depth_img = cv2.flip(depth_img, 1)
        if self.enable_depth_padding:
            depth_img = pad_depth_to_color_size(
                depth_img,
                color_img.shape[0],
                color_img.shape[1],
            )

        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.detector.detectAndCompute(gray, None)
        points_3d = self._points_3d_for_keypoints(keypoints, depth_img)
        self.frame_count += 1

        if descriptors is None or len(keypoints) < self.min_matches:
            if self.publish_debug_image:
                self._publish_debug_image(color_img, keypoints, [], color_msg.header)
            self._store_previous(keypoints, descriptors, points_3d)
            self.get_logger().warn(
                f'视觉里程计特征不足: keypoints={len(keypoints)}',
                throttle_duration_sec=2.0,
            )
            return

        accepted = False
        inlier_matches = []
        if self.prev_descriptors is not None and self.prev_keypoints is not None:
            accepted, inlier_matches = self._estimate_and_publish(
                keypoints,
                descriptors,
                color_msg,
            )

        if self.publish_debug_image:
            self._publish_debug_image(color_img, keypoints, inlier_matches, color_msg.header)

        self._store_previous(keypoints, descriptors, points_3d)
        if accepted:
            self.accepted_count += 1

    def _points_3d_for_keypoints(
        self,
        keypoints: Sequence[cv2.KeyPoint],
        depth_img: np.ndarray,
    ) -> List[Optional[np.ndarray]]:
        points: List[Optional[np.ndarray]] = []
        assert self.fx is not None
        assert self.fy is not None
        assert self.cx is not None
        assert self.cy is not None
        for keypoint in keypoints:
            depth_m = sample_depth_at(
                depth_img,
                keypoint.pt[0],
                keypoint.pt[1],
                self.depth_sample_radius,
                self.depth_scale,
                self.min_depth_m,
                self.max_depth_m,
            )
            if depth_m is None:
                points.append(None)
            else:
                points.append(
                    pixel_to_camera_xyz(
                        keypoint.pt[0],
                        keypoint.pt[1],
                        depth_m,
                        self.fx,
                        self.fy,
                        self.cx,
                        self.cy,
                    )
                )
        return points

    def _estimate_and_publish(
        self,
        keypoints: Sequence[cv2.KeyPoint],
        descriptors: np.ndarray,
        color_msg: Image,
    ) -> Tuple[bool, List[cv2.DMatch]]:
        knn_matches = self.matcher.knnMatch(self.prev_descriptors, descriptors, k=2)
        good_matches = []
        object_points = []
        image_points = []

        for pair in knn_matches:
            if len(pair) < 2:
                continue
            best, second = pair
            if best.distance >= self.match_ratio * second.distance:
                continue
            prev_point = self.prev_points_3d[best.queryIdx]
            if prev_point is None:
                continue
            good_matches.append(best)
            object_points.append(prev_point)
            image_points.append(keypoints[best.trainIdx].pt)

        if len(good_matches) < self.min_matches:
            self.get_logger().warn(
                f'视觉里程计匹配不足: matches={len(good_matches)}',
                throttle_duration_sec=2.0,
            )
            return False, []

        camera_matrix = np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            np.array(object_points, dtype=np.float32),
            np.array(image_points, dtype=np.float32),
            camera_matrix,
            None,
            reprojectionError=self.pnp_reprojection_error,
            iterationsCount=100,
            confidence=0.99,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        if not success or inliers is None or len(inliers) < self.min_inliers:
            inlier_count = 0 if inliers is None else len(inliers)
            self.get_logger().warn(
                f'视觉里程计PnP不稳定: success={success}, inliers={inlier_count}',
                throttle_duration_sec=2.0,
            )
            return False, []

        rotation_curr_prev, _ = cv2.Rodrigues(rvec)
        transform_curr_prev = build_transform(rotation_curr_prev, tvec)
        transform_prev_curr = np.linalg.inv(transform_curr_prev)
        translation = transform_prev_curr[:3, 3]
        angle = math.acos(
            min(1.0, max(-1.0, (np.trace(transform_prev_curr[:3, :3]) - 1.0) / 2.0))
        )

        if np.linalg.norm(translation) > self.max_translation_per_frame_m:
            self.get_logger().warn(
                '视觉里程计单帧位移过大，已丢弃: %.3fm'
                % np.linalg.norm(translation),
                throttle_duration_sec=2.0,
            )
            return False, []
        if abs(angle) > self.max_rotation_per_frame_rad:
            self.get_logger().warn(
                '视觉里程计单帧旋转过大，已丢弃: %.3frad' % angle,
                throttle_duration_sec=2.0,
            )
            return False, []

        self.pose = self.pose @ transform_prev_curr
        self._publish_odometry(color_msg.header.stamp)

        inlier_set = {int(index[0]) for index in inliers}
        inlier_matches = [
            match
            for match_index, match in enumerate(good_matches)
            if match_index in inlier_set
        ]
        self.get_logger().info(
            '视觉里程计更新: matches=%d inliers=%d pos=(%.2f, %.2f, %.2f)'
            % (
                len(good_matches),
                len(inliers),
                self.pose[0, 3],
                self.pose[1, 3],
                self.pose[2, 3],
            ),
            throttle_duration_sec=1.0,
        )
        return True, inlier_matches

    def _publish_odometry(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = float(self.pose[0, 3])
        msg.pose.pose.position.y = float(self.pose[1, 3])
        msg.pose.pose.position.z = float(self.pose[2, 3])
        msg.pose.pose.orientation = quaternion_from_rotation_matrix(self.pose[:3, :3])
        msg.pose.covariance[0] = 0.08 ** 2
        msg.pose.covariance[7] = 0.08 ** 2
        msg.pose.covariance[14] = 0.12 ** 2
        msg.pose.covariance[35] = 0.12 ** 2
        self.odom_pub.publish(msg)

        if self.publish_tf and self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = msg.child_frame_id
            transform.transform.translation.x = msg.pose.pose.position.x
            transform.transform.translation.y = msg.pose.pose.position.y
            transform.transform.translation.z = msg.pose.pose.position.z
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def _publish_debug_image(self, color_img, keypoints, matches, header):
        debug_img = color_img.copy()
        if matches:
            for match in matches[:80]:
                x, y = keypoints[match.trainIdx].pt
                cv2.circle(debug_img, (int(x), int(y)), 3, (0, 255, 0), -1)
        else:
            for keypoint in keypoints[:120]:
                x, y = keypoint.pt
                cv2.circle(debug_img, (int(x), int(y)), 2, (0, 255, 255), -1)
        cv2.putText(
            debug_img,
            (
                f'frames={self.frame_count} accepted={self.accepted_count} '
                f'kp={len(keypoints)} matches={len(matches)}'
            ),
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        msg.header = header
        self.debug_pub.publish(msg)

    def _store_previous(self, keypoints, descriptors, points_3d):
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors
        self.prev_points_3d = points_3d


def main(args=None):
    rclpy.init(args=args)
    node = RgbdVisualOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
