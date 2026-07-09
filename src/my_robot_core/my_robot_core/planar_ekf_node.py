#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


DEFAULT_FILTERED_ODOM_TOPIC = '/odometry/filtered'
DEFAULT_WHEEL_ODOM_TOPIC = '/odom'
DEFAULT_IMU_TOPIC = '/imu/data'
DEFAULT_RGBD_ODOM_TOPIC = '/rgbd/odom'
DEFAULT_USE_VISUAL_ODOM = False
DEFAULT_USE_VISUAL_POSITION = False
DEFAULT_USE_VISUAL_TWIST = True
DEFAULT_USE_VISUAL_YAW = False
DEFAULT_VISUAL_POSITION_STD = 0.80
DEFAULT_VISUAL_TWIST_STD = 0.35
DEFAULT_VISUAL_YAW_STD = 0.80
DEFAULT_VISUAL_FORWARD_AXIS = 'z'
DEFAULT_VISUAL_LEFT_AXIS = '-x'
DEFAULT_VISUAL_MAX_DT = 0.5
DEFAULT_VISUAL_MAX_SPEED_MPS = 0.8
DEFAULT_VISUAL_MAX_DELTA_M = 0.4

STATE_X = 0
STATE_Y = 1
STATE_YAW = 2
STATE_VX = 3
STATE_VY = 4
STATE_WZ = 5
STATE_SIZE = 6


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def axis_delta(delta_xyz, axis_spec):
    """Map a raw visual odom XYZ delta to one planar axis."""
    spec = str(axis_spec).strip().lower()
    sign = 1.0
    if spec.startswith('-'):
        sign = -1.0
        spec = spec[1:]
    elif spec.startswith('+'):
        spec = spec[1:]

    axis_map = {'x': 0, 'y': 1, 'z': 2}
    if spec not in axis_map:
        raise ValueError(f'unsupported visual axis spec: {axis_spec}')
    return sign * float(delta_xyz[axis_map[spec]])


class PlanarEkf:
    """Small 2D EKF for fusing wheel odom, IMU yaw/gyro, and optional RGB-D odom."""

    def __init__(self, process_std):
        self.state = np.zeros(STATE_SIZE, dtype=float)
        self.covariance = np.eye(STATE_SIZE, dtype=float) * 1.0
        self.process_var = np.square(np.array(process_std, dtype=float))
        self.initialized = False
        self.last_time_sec = None

    def initialize(self, time_sec, x=None, y=None, yaw=None, vx=None, vy=None, wz=None):
        if x is not None:
            self.state[STATE_X] = x
        if y is not None:
            self.state[STATE_Y] = y
        if yaw is not None:
            self.state[STATE_YAW] = normalize_angle(yaw)
        if vx is not None:
            self.state[STATE_VX] = vx
        if vy is not None:
            self.state[STATE_VY] = vy
        if wz is not None:
            self.state[STATE_WZ] = wz
        self.last_time_sec = time_sec
        self.initialized = True

    def predict_to(self, time_sec, max_dt):
        if not self.initialized:
            return
        if self.last_time_sec is None:
            self.last_time_sec = time_sec
            return

        dt = time_sec - self.last_time_sec
        if dt <= 0.0:
            return
        dt = min(dt, max_dt)

        yaw = self.state[STATE_YAW]
        vx = self.state[STATE_VX]
        vy = self.state[STATE_VY]
        wz = self.state[STATE_WZ]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        self.state[STATE_X] += (vx * cos_yaw - vy * sin_yaw) * dt
        self.state[STATE_Y] += (vx * sin_yaw + vy * cos_yaw) * dt
        self.state[STATE_YAW] = normalize_angle(yaw + wz * dt)

        f = np.eye(STATE_SIZE, dtype=float)
        f[STATE_X, STATE_YAW] = (-vx * sin_yaw - vy * cos_yaw) * dt
        f[STATE_X, STATE_VX] = cos_yaw * dt
        f[STATE_X, STATE_VY] = -sin_yaw * dt
        f[STATE_Y, STATE_YAW] = (vx * cos_yaw - vy * sin_yaw) * dt
        f[STATE_Y, STATE_VX] = sin_yaw * dt
        f[STATE_Y, STATE_VY] = cos_yaw * dt
        f[STATE_YAW, STATE_WZ] = dt

        q = np.diag(self.process_var * max(dt, 1e-3))
        self.covariance = f @ self.covariance @ f.T + q
        self.last_time_sec += dt

    def update(self, measurement, h, r, angle_measurement_rows=None):
        if not self.initialized:
            return
        z = np.array(measurement, dtype=float)
        h = np.array(h, dtype=float)
        r = np.array(r, dtype=float)

        innovation = z - h @ self.state
        for row in angle_measurement_rows or []:
            innovation[row] = normalize_angle(innovation[row])

        s = h @ self.covariance @ h.T + r
        k = self.covariance @ h.T @ np.linalg.inv(s)
        self.state = self.state + k @ innovation
        self.state[STATE_YAW] = normalize_angle(self.state[STATE_YAW])
        identity = np.eye(STATE_SIZE, dtype=float)
        self.covariance = (identity - k @ h) @ self.covariance


def _stamp_to_sec(msg_stamp, fallback_time):
    if msg_stamp.sec == 0 and msg_stamp.nanosec == 0:
        return fallback_time
    return float(msg_stamp.sec) + float(msg_stamp.nanosec) / 1e9


class PlanarEkfNode(Node):
    def __init__(self):
        super().__init__('planar_ekf_node')

        self.declare_parameter('wheel_odom_topic', DEFAULT_WHEEL_ODOM_TOPIC)
        self.declare_parameter('imu_topic', DEFAULT_IMU_TOPIC)
        self.declare_parameter('rgbd_odom_topic', DEFAULT_RGBD_ODOM_TOPIC)
        self.declare_parameter('filtered_odom_topic', DEFAULT_FILTERED_ODOM_TOPIC)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('max_prediction_dt', 0.2)
        self.declare_parameter(
            'process_std',
            [0.03, 0.03, 0.03, 0.20, 0.20, 0.30],
        )
        self.declare_parameter('wheel_position_std', 0.05)
        self.declare_parameter('wheel_velocity_std', 0.05)
        self.declare_parameter('wheel_yaw_std', 0.20)
        self.declare_parameter('wheel_wz_std', 0.05)
        self.declare_parameter('imu_yaw_std', 0.03)
        self.declare_parameter('imu_wz_std', 0.05)
        self.declare_parameter('visual_position_std', DEFAULT_VISUAL_POSITION_STD)
        self.declare_parameter('visual_twist_std', DEFAULT_VISUAL_TWIST_STD)
        self.declare_parameter('visual_yaw_std', DEFAULT_VISUAL_YAW_STD)
        self.declare_parameter('visual_forward_axis', DEFAULT_VISUAL_FORWARD_AXIS)
        self.declare_parameter('visual_left_axis', DEFAULT_VISUAL_LEFT_AXIS)
        self.declare_parameter('visual_max_dt', DEFAULT_VISUAL_MAX_DT)
        self.declare_parameter('visual_max_speed_mps', DEFAULT_VISUAL_MAX_SPEED_MPS)
        self.declare_parameter('visual_max_delta_m', DEFAULT_VISUAL_MAX_DELTA_M)
        self.declare_parameter('use_wheel_position', True)
        self.declare_parameter('use_wheel_twist', True)
        self.declare_parameter('use_wheel_yaw_until_imu', True)
        self.declare_parameter('use_wheel_wz_until_imu', True)
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('use_imu_wz', True)
        self.declare_parameter('use_visual_odom', DEFAULT_USE_VISUAL_ODOM)
        self.declare_parameter('use_visual_position', DEFAULT_USE_VISUAL_POSITION)
        self.declare_parameter('use_visual_twist', DEFAULT_USE_VISUAL_TWIST)
        self.declare_parameter('use_visual_yaw', DEFAULT_USE_VISUAL_YAW)

        self.wheel_odom_topic = self.get_parameter('wheel_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.rgbd_odom_topic = self.get_parameter('rgbd_odom_topic').value
        self.filtered_odom_topic = self.get_parameter('filtered_odom_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.max_prediction_dt = self.get_parameter('max_prediction_dt').value

        process_std = self.get_parameter('process_std').value
        self.filter = PlanarEkf(process_std)
        self.imu_received = False
        self.visual_warning_printed = False

        self.wheel_position_var = self.get_parameter('wheel_position_std').value ** 2
        self.wheel_velocity_var = self.get_parameter('wheel_velocity_std').value ** 2
        self.wheel_yaw_var = self.get_parameter('wheel_yaw_std').value ** 2
        self.wheel_wz_var = self.get_parameter('wheel_wz_std').value ** 2
        self.imu_yaw_var = self.get_parameter('imu_yaw_std').value ** 2
        self.imu_wz_var = self.get_parameter('imu_wz_std').value ** 2
        self.visual_position_var = self.get_parameter('visual_position_std').value ** 2
        self.visual_twist_var = self.get_parameter('visual_twist_std').value ** 2
        self.visual_yaw_var = self.get_parameter('visual_yaw_std').value ** 2
        self.visual_forward_axis = self.get_parameter('visual_forward_axis').value
        self.visual_left_axis = self.get_parameter('visual_left_axis').value
        self.visual_max_dt = float(self.get_parameter('visual_max_dt').value)
        self.visual_max_speed_mps = float(
            self.get_parameter('visual_max_speed_mps').value
        )
        self.visual_max_delta_m = float(self.get_parameter('visual_max_delta_m').value)

        self.use_wheel_position = self.get_parameter('use_wheel_position').value
        self.use_wheel_twist = self.get_parameter('use_wheel_twist').value
        self.use_wheel_yaw_until_imu = self.get_parameter(
            'use_wheel_yaw_until_imu'
        ).value
        self.use_wheel_wz_until_imu = self.get_parameter(
            'use_wheel_wz_until_imu'
        ).value
        self.use_imu_yaw = self.get_parameter('use_imu_yaw').value
        self.use_imu_wz = self.get_parameter('use_imu_wz').value
        self.use_visual_odom = self.get_parameter('use_visual_odom').value
        self.use_visual_position = self.get_parameter('use_visual_position').value
        self.use_visual_twist = self.get_parameter('use_visual_twist').value
        self.use_visual_yaw = self.get_parameter('use_visual_yaw').value
        self.prev_visual_stamp_sec = None
        self.prev_visual_position = None
        self.visual_reject_warning_printed = False

        self.odom_pub = self.create_publisher(Odometry, self.filtered_odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Odometry, self.wheel_odom_topic, self.wheel_odom_callback, 20)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 50)
        self.create_subscription(Odometry, self.rgbd_odom_topic, self.rgbd_odom_callback, 10)

        publish_rate = max(self.get_parameter('publish_rate_hz').value, 1.0)
        self.create_timer(1.0 / publish_rate, self.publish_filtered_odom)
        self.get_logger().info(
            f"EKF已启动: {self.wheel_odom_topic} + {self.imu_topic} -> "
            f"{self.filtered_odom_topic}, publish_tf={self.publish_tf}, "
            f"use_visual_odom={self.use_visual_odom}, "
            f"visual_twist={self.use_visual_twist}, "
            f"visual_position={self.use_visual_position}, "
            f"visual_yaw={self.use_visual_yaw}"
        )

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _predict_to_msg_time(self, msg_stamp):
        stamp_sec = _stamp_to_sec(msg_stamp, self._now_sec())
        self.filter.predict_to(stamp_sec, self.max_prediction_dt)
        return stamp_sec

    def wheel_odom_callback(self, msg):
        stamp_sec = _stamp_to_sec(msg.header.stamp, self._now_sec())
        wheel_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if not self.filter.initialized:
            self.filter.initialize(
                stamp_sec,
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                yaw=wheel_yaw,
                vx=msg.twist.twist.linear.x,
                vy=msg.twist.twist.linear.y,
                wz=msg.twist.twist.angular.z,
            )
        else:
            self.filter.predict_to(stamp_sec, self.max_prediction_dt)

        rows = []
        measurement = []
        variances = []
        angle_rows = []

        if self.use_wheel_position:
            rows.extend([STATE_X, STATE_Y])
            measurement.extend([msg.pose.pose.position.x, msg.pose.pose.position.y])
            variances.extend([self.wheel_position_var, self.wheel_position_var])
        if self.use_wheel_twist:
            rows.extend([STATE_VX, STATE_VY])
            measurement.extend([msg.twist.twist.linear.x, msg.twist.twist.linear.y])
            variances.extend([self.wheel_velocity_var, self.wheel_velocity_var])
        if self.use_wheel_yaw_until_imu and not self.imu_received:
            angle_rows.append(len(measurement))
            rows.append(STATE_YAW)
            measurement.append(wheel_yaw)
            variances.append(self.wheel_yaw_var)
        if self.use_wheel_wz_until_imu and not self.imu_received:
            rows.append(STATE_WZ)
            measurement.append(msg.twist.twist.angular.z)
            variances.append(self.wheel_wz_var)

        self._apply_measurement(rows, measurement, variances, angle_rows)

    def imu_callback(self, msg):
        stamp_sec = _stamp_to_sec(msg.header.stamp, self._now_sec())
        imu_yaw = yaw_from_quaternion(msg.orientation)

        if not self.filter.initialized:
            self.filter.initialize(
                stamp_sec,
                yaw=imu_yaw,
                wz=msg.angular_velocity.z,
            )
        else:
            self.filter.predict_to(stamp_sec, self.max_prediction_dt)

        rows = []
        measurement = []
        variances = []
        angle_rows = []

        if self.use_imu_yaw and msg.orientation_covariance[0] != -1.0:
            angle_rows.append(len(measurement))
            rows.append(STATE_YAW)
            measurement.append(imu_yaw)
            variances.append(self.imu_yaw_var)
        if self.use_imu_wz and msg.angular_velocity_covariance[0] != -1.0:
            rows.append(STATE_WZ)
            measurement.append(msg.angular_velocity.z)
            variances.append(self.imu_wz_var)

        self._apply_measurement(rows, measurement, variances, angle_rows)
        self.imu_received = True

    def rgbd_odom_callback(self, msg):
        if not self.use_visual_odom:
            return
        if msg.header.frame_id and msg.header.frame_id != self.odom_frame_id:
            if not self.visual_warning_printed:
                self.get_logger().warn(
                    f"收到 {self.rgbd_odom_topic} frame_id={msg.header.frame_id}，"
                    f"与EKF odom_frame_id={self.odom_frame_id} 不一致；"
                    "默认仅按视觉增量速度低权重融合，不直接使用视觉绝对位置。"
                )
                self.visual_warning_printed = True

        stamp_sec = _stamp_to_sec(msg.header.stamp, self._now_sec())
        current_visual_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ], dtype=float)
        if not self.filter.initialized:
            # Wheel odom / IMU own the EKF initial pose. Visual odom starts in
            # its own local camera frame, so the first visual frame is only a
            # reference for later delta/twist updates.
            self._store_visual_reference(stamp_sec, current_visual_position)
            return
        else:
            self.filter.predict_to(stamp_sec, self.max_prediction_dt)

        rows = []
        measurement = []
        variances = []
        angle_rows = []

        if self.use_visual_position:
            rows.extend([STATE_X, STATE_Y])
            measurement.extend([msg.pose.pose.position.x, msg.pose.pose.position.y])
            variances.extend([self.visual_position_var, self.visual_position_var])

        if self.use_visual_twist:
            twist_measurement = self._visual_twist_measurement(
                stamp_sec,
                current_visual_position,
            )
            if twist_measurement is not None:
                visual_vx, visual_vy = twist_measurement
                rows.extend([STATE_VX, STATE_VY])
                measurement.extend([visual_vx, visual_vy])
                variances.extend([self.visual_twist_var, self.visual_twist_var])

        if self.use_visual_yaw:
            angle_rows.append(len(measurement))
            rows.append(STATE_YAW)
            measurement.append(yaw_from_quaternion(msg.pose.pose.orientation))
            variances.append(self.visual_yaw_var)

        self._apply_measurement(rows, measurement, variances, angle_rows)
        self._store_visual_reference(stamp_sec, current_visual_position)

    def _store_visual_reference(self, stamp_sec, position):
        self.prev_visual_stamp_sec = stamp_sec
        self.prev_visual_position = position

    def _visual_twist_measurement(self, stamp_sec, current_position):
        if self.prev_visual_stamp_sec is None or self.prev_visual_position is None:
            self._store_visual_reference(stamp_sec, current_position)
            return None

        dt = stamp_sec - self.prev_visual_stamp_sec
        if dt <= 0.0 or dt > self.visual_max_dt:
            self._store_visual_reference(stamp_sec, current_position)
            return None

        delta = current_position - self.prev_visual_position
        delta_norm = float(np.linalg.norm(delta))
        if delta_norm > self.visual_max_delta_m:
            self.get_logger().warn(
                'RGB-D视觉里程计增量过大，跳过本次EKF视觉更新: '
                'delta=%.3fm dt=%.3fs'
                % (delta_norm, dt),
                throttle_duration_sec=2.0,
            )
            return None

        try:
            visual_vx = axis_delta(delta, self.visual_forward_axis) / dt
            visual_vy = axis_delta(delta, self.visual_left_axis) / dt
        except ValueError as exc:
            if not self.visual_reject_warning_printed:
                self.get_logger().warn(str(exc))
                self.visual_reject_warning_printed = True
            return None

        speed = math.hypot(visual_vx, visual_vy)
        if speed > self.visual_max_speed_mps:
            self.get_logger().warn(
                'RGB-D视觉里程计速度过大，跳过本次EKF视觉更新: '
                'vx=%.3f vy=%.3f speed=%.3fm/s'
                % (visual_vx, visual_vy, speed),
                throttle_duration_sec=2.0,
            )
            return None

        return visual_vx, visual_vy

    def _apply_measurement(self, rows, measurement, variances, angle_rows):
        if not rows:
            return
        h = np.zeros((len(rows), STATE_SIZE), dtype=float)
        for index, state_index in enumerate(rows):
            h[index, state_index] = 1.0
        r = np.diag(variances)
        self.filter.update(measurement, h, r, angle_rows)

    def publish_filtered_odom(self):
        if not self.filter.initialized:
            return

        now = self.get_clock().now()
        self.filter.predict_to(now.nanoseconds / 1e9, self.max_prediction_dt)
        state = self.filter.state

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id
        msg.pose.pose.position.x = float(state[STATE_X])
        msg.pose.pose.position.y = float(state[STATE_Y])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, state[STATE_YAW])
        msg.twist.twist.linear.x = float(state[STATE_VX])
        msg.twist.twist.linear.y = float(state[STATE_VY])
        msg.twist.twist.angular.z = float(state[STATE_WZ])

        covariance = self.filter.covariance
        msg.pose.covariance[0] = float(covariance[STATE_X, STATE_X])
        msg.pose.covariance[7] = float(covariance[STATE_Y, STATE_Y])
        msg.pose.covariance[35] = float(covariance[STATE_YAW, STATE_YAW])
        msg.twist.covariance[0] = float(covariance[STATE_VX, STATE_VX])
        msg.twist.covariance[7] = float(covariance[STATE_VY, STATE_VY])
        msg.twist.covariance[35] = float(covariance[STATE_WZ, STATE_WZ])
        self.odom_pub.publish(msg)

        if self.publish_tf and self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = msg.pose.pose.position.x
            transform.transform.translation.y = msg.pose.pose.position.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = PlanarEkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
