#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan


TILTED_SCAN_MODE_SAFETY_ONLY = 'safety_only'
TILTED_SCAN_MODE_ALL_MAPS = 'all_maps'
TILTED_SCAN_MODES = {
    TILTED_SCAN_MODE_SAFETY_ONLY,
    TILTED_SCAN_MODE_ALL_MAPS,
}

# Field-tuning default. Unit: meters.
# This controls the max finite LaserScan range kept while the chassis is tilted.
# Change this default and rebuild if you want a new project-wide startup value.
# Runtime launch/ros2 param overrides still take priority over this value.
DEFAULT_TILTED_SLOPE_SCAN_RANGE_M = 0.70
SAFETY_RANGE_EPSILON_M = 1e-6



class LaserFilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter_node')

        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_filtered')
        self.declare_parameter('safety_scan_topic', '/scan_safety')
        self.declare_parameter('tilt_topic', '/base/tilt')
        self.declare_parameter('drop_roll_threshold_deg', 2.0)
        self.declare_parameter('drop_pitch_threshold_deg', 2.0)
        self.declare_parameter('resume_roll_threshold_deg', 1.0)
        self.declare_parameter('resume_pitch_threshold_deg', 1.0)
        self.declare_parameter('stable_frames_required', 3)
        self.declare_parameter(
            'tilted_safety_max_range',
            DEFAULT_TILTED_SLOPE_SCAN_RANGE_M,
        )
        self.declare_parameter('tilted_scan_mode', TILTED_SCAN_MODE_ALL_MAPS)
        self.declare_parameter('pass_through_without_tilt', True)
        self.declare_parameter('enable_tilt_gate', True)
        self.declare_parameter('startup_tilt_bootstrap_enabled', True)
        self.declare_parameter('startup_tilt_bootstrap_max_frames', 3)
        self.declare_parameter('startup_tilt_bootstrap_max_duration_sec', 2.0)
        self.declare_parameter('startup_tilt_bootstrap_max_angle_deg', 5.0)
        self.declare_parameter('roll_offset_deg', 0.0)
        self.declare_parameter('pitch_offset_deg', 0.0)

        self.input_scan_topic = self.get_parameter('input_scan_topic').value
        self.output_scan_topic = self.get_parameter('output_scan_topic').value
        self.safety_scan_topic = self.get_parameter('safety_scan_topic').value
        self.tilt_topic = self.get_parameter('tilt_topic').value
        self._refresh_runtime_parameters()

        self.latest_raw_roll = None
        self.latest_raw_pitch = None
        self.latest_roll = None
        self.latest_pitch = None
        self.blocking_scan = False
        self.stable_scan_count = 0
        self.has_published_level_scan = False
        self.startup_time = time.monotonic()
        self.startup_bootstrap_frames_used = 0

        self.scan_pub = self.create_publisher(LaserScan, self.output_scan_topic, 10)
        self.safety_scan_pub = self.create_publisher(
            LaserScan, self.safety_scan_topic, 10
        )
        self.create_subscription(Vector3Stamped, self.tilt_topic, self.on_tilt, 10)
        self.create_subscription(LaserScan, self.input_scan_topic, self.on_scan, 10)
        self.create_timer(1.0, self.log_status)
        self.add_on_set_parameters_callback(self.on_parameter_update)

        self.get_logger().info(
            'laser_filter_node started: '
            f'{self.input_scan_topic} -> trusted={self.output_scan_topic}, '
            f'safety={self.safety_scan_topic}, '
            f'tilt={self.tilt_topic}, '
            f'enable_tilt_gate={self.enable_tilt_gate}, '
            f'tilted_safety_max_range={self.tilted_safety_max_range:.2f}m, '
            f'tilted_scan_mode={self.tilted_scan_mode}, '
            f'startup_bootstrap={self.startup_tilt_bootstrap_enabled}, '
            f'roll_offset={math.degrees(self.roll_offset):.2f}deg, '
            f'pitch_offset={math.degrees(self.pitch_offset):.2f}deg'
        )

    def _refresh_runtime_parameters(self):
        self.drop_roll_threshold = math.radians(
            self.get_parameter('drop_roll_threshold_deg').value
        )
        self.drop_pitch_threshold = math.radians(
            self.get_parameter('drop_pitch_threshold_deg').value
        )
        self.resume_roll_threshold = math.radians(
            self.get_parameter('resume_roll_threshold_deg').value
        )
        self.resume_pitch_threshold = math.radians(
            self.get_parameter('resume_pitch_threshold_deg').value
        )
        self.stable_frames_required = max(
            1, int(self.get_parameter('stable_frames_required').value)
        )
        self.tilted_safety_max_range = float(
            self.get_parameter('tilted_safety_max_range').value
        )
        self.tilted_scan_mode = self.get_parameter('tilted_scan_mode').value
        if self.tilted_scan_mode not in TILTED_SCAN_MODES:
            self.get_logger().warn(
                f'Invalid tilted_scan_mode={self.tilted_scan_mode}, '
                f'falling back to {TILTED_SCAN_MODE_ALL_MAPS}'
            )
            self.tilted_scan_mode = TILTED_SCAN_MODE_ALL_MAPS
        self.pass_through_without_tilt = self.get_parameter(
            'pass_through_without_tilt'
        ).value
        self.enable_tilt_gate = self.get_parameter('enable_tilt_gate').value
        self.startup_tilt_bootstrap_enabled = self.get_parameter(
            'startup_tilt_bootstrap_enabled'
        ).value
        self.startup_tilt_bootstrap_max_frames = max(
            0, int(self.get_parameter('startup_tilt_bootstrap_max_frames').value)
        )
        self.startup_tilt_bootstrap_max_duration_sec = float(
            self.get_parameter('startup_tilt_bootstrap_max_duration_sec').value
        )
        self.startup_tilt_bootstrap_max_angle = math.radians(
            self.get_parameter('startup_tilt_bootstrap_max_angle_deg').value
        )
        self.roll_offset = math.radians(self.get_parameter('roll_offset_deg').value)
        self.pitch_offset = math.radians(self.get_parameter('pitch_offset_deg').value)

    def on_parameter_update(self, params):
        numeric_params = {
            'drop_roll_threshold_deg',
            'drop_pitch_threshold_deg',
            'resume_roll_threshold_deg',
            'resume_pitch_threshold_deg',
            'tilted_safety_max_range',
            'startup_tilt_bootstrap_max_duration_sec',
            'startup_tilt_bootstrap_max_angle_deg',
            'roll_offset_deg',
            'pitch_offset_deg',
        }
        int_params = {
            'stable_frames_required': 1,
            'startup_tilt_bootstrap_max_frames': 0,
        }
        bool_params = {
            'pass_through_without_tilt',
            'enable_tilt_gate',
            'startup_tilt_bootstrap_enabled',
        }
        string_params = {
            'tilted_scan_mode',
        }
        restart_required_params = {
            'input_scan_topic',
            'output_scan_topic',
            'safety_scan_topic',
            'tilt_topic',
        }

        for param in params:
            if param.name in numeric_params:
                if param.type_ not in (
                    Parameter.Type.INTEGER,
                    Parameter.Type.DOUBLE,
                ):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{param.name} must be numeric',
                    )
                if param.name == 'tilted_safety_max_range' and param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='tilted_safety_max_range must be > 0.0',
                    )
                is_bootstrap_duration = (
                    param.name == 'startup_tilt_bootstrap_max_duration_sec'
                )
                if is_bootstrap_duration and param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='startup_tilt_bootstrap_max_duration_sec must be >= 0.0',
                    )
                is_bootstrap_angle = (
                    param.name == 'startup_tilt_bootstrap_max_angle_deg'
                )
                if is_bootstrap_angle and param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='startup_tilt_bootstrap_max_angle_deg must be > 0.0',
                    )
            elif param.name in int_params:
                try:
                    int_value = int(param.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{param.name} must be an integer',
                    )
                min_value = int_params[param.name]
                if int_value < min_value:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{param.name} must be >= {min_value}',
                    )
            elif param.name in bool_params:
                if param.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{param.name} must be bool',
                    )
            elif param.name in string_params:
                if param.type_ != Parameter.Type.STRING:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{param.name} must be string',
                    )
                if param.value not in TILTED_SCAN_MODES:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f'{param.name} must be one of '
                            f'{sorted(TILTED_SCAN_MODES)}'
                        ),
                    )
            elif param.name in restart_required_params:
                return SetParametersResult(
                    successful=False,
                    reason=f'{param.name} requires restarting laser_filter_node',
                )

        for param in params:
            if param.name == 'drop_roll_threshold_deg':
                self.drop_roll_threshold = math.radians(float(param.value))
            elif param.name == 'drop_pitch_threshold_deg':
                self.drop_pitch_threshold = math.radians(float(param.value))
            elif param.name == 'resume_roll_threshold_deg':
                self.resume_roll_threshold = math.radians(float(param.value))
            elif param.name == 'resume_pitch_threshold_deg':
                self.resume_pitch_threshold = math.radians(float(param.value))
            elif param.name == 'stable_frames_required':
                self.stable_frames_required = int(param.value)
            elif param.name == 'tilted_safety_max_range':
                self.tilted_safety_max_range = float(param.value)
            elif param.name == 'pass_through_without_tilt':
                self.pass_through_without_tilt = bool(param.value)
            elif param.name == 'enable_tilt_gate':
                self.enable_tilt_gate = bool(param.value)
                if not self.enable_tilt_gate:
                    self.blocking_scan = False
                    self.stable_scan_count = 0
            elif param.name == 'tilted_scan_mode':
                self.tilted_scan_mode = str(param.value)
            elif param.name == 'startup_tilt_bootstrap_enabled':
                self.startup_tilt_bootstrap_enabled = bool(param.value)
            elif param.name == 'startup_tilt_bootstrap_max_frames':
                self.startup_tilt_bootstrap_max_frames = int(param.value)
            elif param.name == 'startup_tilt_bootstrap_max_duration_sec':
                self.startup_tilt_bootstrap_max_duration_sec = float(param.value)
            elif param.name == 'startup_tilt_bootstrap_max_angle_deg':
                self.startup_tilt_bootstrap_max_angle = math.radians(float(param.value))
            elif param.name == 'roll_offset_deg':
                self.roll_offset = math.radians(float(param.value))
            elif param.name == 'pitch_offset_deg':
                self.pitch_offset = math.radians(float(param.value))

        if self.latest_raw_roll is not None and self.latest_raw_pitch is not None:
            self.latest_roll = self.latest_raw_roll - self.roll_offset
            self.latest_pitch = self.latest_raw_pitch - self.pitch_offset

        self.get_logger().info(
            'laser_filter parameters updated: '
            f'enable_tilt_gate={self.enable_tilt_gate}, '
            f'drop=({math.degrees(self.drop_roll_threshold):.2f},'
            f'{math.degrees(self.drop_pitch_threshold):.2f})deg, '
            f'resume=({math.degrees(self.resume_roll_threshold):.2f},'
            f'{math.degrees(self.resume_pitch_threshold):.2f})deg, '
            f'tilted_safety_max_range={self.tilted_safety_max_range:.2f}m, '
            f'tilted_scan_mode={self.tilted_scan_mode}, '
            f'startup_bootstrap={self.startup_tilt_bootstrap_enabled}, '
            f'startup_bootstrap_frames={self.startup_tilt_bootstrap_max_frames}, '
            f'startup_bootstrap_duration='
            f'{self.startup_tilt_bootstrap_max_duration_sec:.2f}s, '
            f'startup_bootstrap_angle='
            f'{math.degrees(self.startup_tilt_bootstrap_max_angle):.2f}deg, '
            f'offset=({math.degrees(self.roll_offset):.2f},'
            f'{math.degrees(self.pitch_offset):.2f})deg'
        )
        return SetParametersResult(successful=True)

    def on_tilt(self, msg: Vector3Stamped):
        self.latest_raw_roll = msg.vector.x
        self.latest_raw_pitch = msg.vector.y
        self.latest_roll = msg.vector.x - self.roll_offset
        self.latest_pitch = msg.vector.y - self.pitch_offset

    def _is_valid_safety_range(self, value, range_min):
        if not math.isfinite(value):
            return False
        if value < range_min:
            return False
        return value <= self.tilted_safety_max_range + SAFETY_RANGE_EPSILON_M

    def _make_tilted_safety_scan(self, msg: LaserScan):
        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.scan_time = msg.scan_time
        scan.range_min = msg.range_min
        scan.range_max = msg.range_max
        scan.ranges = [
            value
            if self._is_valid_safety_range(value, msg.range_min)
            else math.nan
            for value in msg.ranges
        ]
        scan.intensities = list(msg.intensities)
        return scan

    def _publish_trusted_scan(self, msg: LaserScan, is_level_scan=False):
        self.scan_pub.publish(msg)
        if is_level_scan:
            self.has_published_level_scan = True

    def _publish_safety_scan(self, msg: LaserScan, limit_to_near=False):
        if limit_to_near:
            self.safety_scan_pub.publish(self._make_tilted_safety_scan(msg))
        else:
            self.safety_scan_pub.publish(msg)

    def _publish_tilted_trusted_scan(self, msg: LaserScan):
        if self.tilted_scan_mode != TILTED_SCAN_MODE_ALL_MAPS:
            return False

        self._publish_trusted_scan(self._make_tilted_safety_scan(msg))
        return True

    def _startup_bootstrap_allowed(self, abs_roll, abs_pitch):
        if not self.startup_tilt_bootstrap_enabled:
            return False
        if self.has_published_level_scan:
            return False
        if self.startup_bootstrap_frames_used >= self.startup_tilt_bootstrap_max_frames:
            return False
        elapsed_sec = time.monotonic() - self.startup_time
        if elapsed_sec > self.startup_tilt_bootstrap_max_duration_sec:
            return False
        max_tilt = max(abs_roll, abs_pitch)
        return max_tilt <= self.startup_tilt_bootstrap_max_angle

    def _publish_startup_bootstrap_scan(self, msg: LaserScan):
        self.startup_bootstrap_frames_used += 1
        self.get_logger().warn(
            'Startup tilt bootstrap publishing trusted scan: '
            f'frame={self.startup_bootstrap_frames_used}/'
            f'{self.startup_tilt_bootstrap_max_frames}, '
            f'roll={math.degrees(self.latest_roll):.2f}deg, '
            f'pitch={math.degrees(self.latest_pitch):.2f}deg, '
            f'limit={math.degrees(self.startup_tilt_bootstrap_max_angle):.2f}deg',
            throttle_duration_sec=1.0,
        )
        self._publish_trusted_scan(msg, is_level_scan=False)

    def on_scan(self, msg: LaserScan):
        if not self.enable_tilt_gate:
            if self.blocking_scan:
                self.blocking_scan = False
                self.stable_scan_count = 0
            self._publish_trusted_scan(msg, is_level_scan=True)
            self._publish_safety_scan(msg)
            return

        if self.latest_roll is None or self.latest_pitch is None:
            if self.pass_through_without_tilt:
                self._publish_trusted_scan(msg, is_level_scan=True)
                self._publish_safety_scan(msg)
            else:
                self.get_logger().warn(
                    'No tilt data yet, dropping LaserScan frame.',
                    throttle_duration_sec=2.0,
                )
            return

        abs_roll = abs(self.latest_roll)
        abs_pitch = abs(self.latest_pitch)

        if not self.blocking_scan:
            if abs_roll > self.drop_roll_threshold or abs_pitch > self.drop_pitch_threshold:
                self.blocking_scan = True
                self.stable_scan_count = 0
                self._publish_safety_scan(msg, limit_to_near=True)
                if self._publish_tilted_trusted_scan(msg):
                    self.get_logger().warn(
                        'Publishing near-only trusted LaserScan on slope: '
                        f'roll={math.degrees(self.latest_roll):.2f}deg, '
                        f'pitch={math.degrees(self.latest_pitch):.2f}deg, '
                        f'range={self.tilted_safety_max_range:.2f}m, '
                        f'mode={self.tilted_scan_mode}',
                        throttle_duration_sec=1.0,
                    )
                    return
                if self._startup_bootstrap_allowed(abs_roll, abs_pitch):
                    self._publish_startup_bootstrap_scan(msg)
                    return
                self.get_logger().warn(
                    'Blocking trusted LaserScan on slope, publishing safety scan only: '
                    f'roll={math.degrees(self.latest_roll):.2f}deg, '
                    f'pitch={math.degrees(self.latest_pitch):.2f}deg, '
                    f'safety_range={self.tilted_safety_max_range:.2f}m',
                    throttle_duration_sec=1.0,
                )
                return

            self._publish_trusted_scan(msg, is_level_scan=True)
            self._publish_safety_scan(msg)
            return

        if abs_roll < self.resume_roll_threshold and abs_pitch < self.resume_pitch_threshold:
            self.stable_scan_count += 1
            if self.stable_scan_count >= self.stable_frames_required:
                self.blocking_scan = False
                self.stable_scan_count = 0
                self.get_logger().info('LaserScan publishing resumed after tilt recovery.')
                self._publish_trusted_scan(msg, is_level_scan=True)
                self._publish_safety_scan(msg)
            else:
                self._publish_safety_scan(msg)
                self._publish_tilted_trusted_scan(msg)
            return

        self.stable_scan_count = 0
        self._publish_safety_scan(msg, limit_to_near=True)
        if self._publish_tilted_trusted_scan(msg):
            return
        if self._startup_bootstrap_allowed(abs_roll, abs_pitch):
            self._publish_startup_bootstrap_scan(msg)

    def log_status(self):
        if self.latest_roll is None or self.latest_pitch is None:
            self.get_logger().info(
                'laser_filter status: waiting for tilt data, '
                f'enable_tilt_gate={self.enable_tilt_gate}, '
                f'blocking={self.blocking_scan}, '
                f'tilted_scan_mode={self.tilted_scan_mode}, '
                f'trusted={self.output_scan_topic}, '
                f'safety={self.safety_scan_topic}'
            )
            return

        self.get_logger().info(
            'laser_filter status: '
            f'roll={math.degrees(self.latest_roll):.2f}deg, '
            f'pitch={math.degrees(self.latest_pitch):.2f}deg, '
            f'raw_roll={math.degrees(self.latest_raw_roll):.2f}deg, '
            f'raw_pitch={math.degrees(self.latest_raw_pitch):.2f}deg, '
            f'enable_tilt_gate={self.enable_tilt_gate}, '
            f'blocking={self.blocking_scan}, '
            f'tilted_scan_mode={self.tilted_scan_mode}, '
            f'stable_frames={self.stable_scan_count}, '
            f'tilted_safety_max_range={self.tilted_safety_max_range:.2f}m, '
            f'level_scan_ready={self.has_published_level_scan}, '
            f'startup_bootstrap_frames={self.startup_bootstrap_frames_used}/'
            f'{self.startup_tilt_bootstrap_max_frames}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LaserFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
