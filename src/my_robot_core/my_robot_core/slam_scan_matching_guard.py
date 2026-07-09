#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rcl_interfaces.msg import Parameter as RosParameter
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node


# Field-tuning default.
# False means the ramp scan-matching toggle node is disabled unless explicitly
# enabled with enable_slam_scan_matching_guard:=true or enable_guard:=true.
DEFAULT_ENABLE_SLAM_SCAN_MATCHING_GUARD = False


class SlamScanMatchingGuard(Node):
    def __init__(self):
        super().__init__('slam_scan_matching_guard')

        self.declare_parameter(
            'enable_guard',
            DEFAULT_ENABLE_SLAM_SCAN_MATCHING_GUARD,
        )
        self.declare_parameter('tilt_topic', '/base/tilt')
        self.declare_parameter('slam_toolbox_set_parameters_service',
                               '/slam_toolbox/set_parameters')
        self.declare_parameter('drop_roll_threshold_deg', 2.0)
        self.declare_parameter('drop_pitch_threshold_deg', 2.0)
        self.declare_parameter('resume_roll_threshold_deg', 1.0)
        self.declare_parameter('resume_pitch_threshold_deg', 1.0)
        self.declare_parameter('stable_frames_required', 3)
        self.declare_parameter('roll_offset_deg', 0.0)
        self.declare_parameter('pitch_offset_deg', 0.0)
        self.declare_parameter('level_scan_matching_enabled', True)
        self.declare_parameter('tilted_scan_matching_enabled', False)
        self.declare_parameter('retry_period_sec', 0.5)

        self.enable_guard = self.get_parameter('enable_guard').value
        self.tilt_topic = self.get_parameter('tilt_topic').value
        self.slam_set_parameters_service = self.get_parameter(
            'slam_toolbox_set_parameters_service'
        ).value
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
        self.roll_offset = math.radians(self.get_parameter('roll_offset_deg').value)
        self.pitch_offset = math.radians(self.get_parameter('pitch_offset_deg').value)
        self.level_scan_matching_enabled = bool(
            self.get_parameter('level_scan_matching_enabled').value
        )
        self.tilted_scan_matching_enabled = bool(
            self.get_parameter('tilted_scan_matching_enabled').value
        )
        self.retry_period_sec = max(
            0.1, float(self.get_parameter('retry_period_sec').value)
        )

        self.latest_raw_roll = None
        self.latest_raw_pitch = None
        self.latest_roll = None
        self.latest_pitch = None
        self.tilted = False
        self.stable_frame_count = 0
        self.desired_scan_matching_enabled = self.level_scan_matching_enabled
        self.applied_scan_matching_enabled = None
        self.pending_future = None
        self.pending_value = None

        self.param_client = self.create_client(
            SetParameters,
            self.slam_set_parameters_service,
        )
        self.create_subscription(Vector3Stamped, self.tilt_topic, self.on_tilt, 10)
        self.create_timer(self.retry_period_sec, self.sync_scan_matching_parameter)
        self.create_timer(1.0, self.log_status)

        self.get_logger().info(
            'slam_scan_matching_guard started: '
            f'enable_guard={self.enable_guard}, '
            f'tilt={self.tilt_topic}, '
            f'service={self.slam_set_parameters_service}, '
            f'drop=({math.degrees(self.drop_roll_threshold):.2f},'
            f'{math.degrees(self.drop_pitch_threshold):.2f})deg, '
            f'resume=({math.degrees(self.resume_roll_threshold):.2f},'
            f'{math.degrees(self.resume_pitch_threshold):.2f})deg, '
            f'level_use_scan_matching={self.level_scan_matching_enabled}, '
            f'tilted_use_scan_matching={self.tilted_scan_matching_enabled}, '
            f'offset=({math.degrees(self.roll_offset):.2f},'
            f'{math.degrees(self.pitch_offset):.2f})deg'
        )

    def on_tilt(self, msg: Vector3Stamped):
        self.latest_raw_roll = msg.vector.x
        self.latest_raw_pitch = msg.vector.y
        self.latest_roll = msg.vector.x - self.roll_offset
        self.latest_pitch = msg.vector.y - self.pitch_offset

        state_changed = self._update_tilt_state(
            abs(self.latest_roll),
            abs(self.latest_pitch),
        )
        if state_changed:
            state = 'tilted' if self.tilted else 'level'
            self.get_logger().warn(
                'SLAM scan matching guard state changed: '
                f'state={state}, '
                f'roll={math.degrees(self.latest_roll):.2f}deg, '
                f'pitch={math.degrees(self.latest_pitch):.2f}deg, '
                f'use_scan_matching={self.desired_scan_matching_enabled}'
            )
            self.sync_scan_matching_parameter()

    def _update_tilt_state(self, abs_roll, abs_pitch):
        previous_tilted = self.tilted

        if not self.tilted:
            if abs_roll > self.drop_roll_threshold or abs_pitch > self.drop_pitch_threshold:
                self.tilted = True
                self.stable_frame_count = 0
        else:
            if abs_roll < self.resume_roll_threshold and abs_pitch < self.resume_pitch_threshold:
                self.stable_frame_count += 1
                if self.stable_frame_count >= self.stable_frames_required:
                    self.tilted = False
                    self.stable_frame_count = 0
            else:
                self.stable_frame_count = 0

        self.desired_scan_matching_enabled = (
            self.tilted_scan_matching_enabled
            if self.tilted
            else self.level_scan_matching_enabled
        )
        return previous_tilted != self.tilted

    def _make_scan_matching_request(self, enabled):
        request = SetParameters.Request()
        request.parameters = [
            RosParameter(
                name='use_scan_matching',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL,
                    bool_value=bool(enabled),
                ),
            )
        ]
        return request

    def sync_scan_matching_parameter(self):
        if not self.enable_guard:
            return

        if self.pending_future is not None:
            if not self.pending_future.done():
                return
            self._handle_pending_response()

        if self.applied_scan_matching_enabled == self.desired_scan_matching_enabled:
            return

        if not self.param_client.service_is_ready():
            if not self.param_client.wait_for_service(timeout_sec=0.0):
                self.get_logger().warn(
                    'Waiting for slam_toolbox set_parameters service: '
                    f'{self.slam_set_parameters_service}',
                    throttle_duration_sec=2.0,
                )
                return

        request = self._make_scan_matching_request(
            self.desired_scan_matching_enabled
        )
        self.pending_value = self.desired_scan_matching_enabled
        self.pending_future = self.param_client.call_async(request)

    def _handle_pending_response(self):
        expected_value = self.pending_value
        future = self.pending_future
        self.pending_value = None
        self.pending_future = None

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(
                'Failed to set slam_toolbox use_scan_matching: '
                f'{exc}',
                throttle_duration_sec=2.0,
            )
            return

        if not response.results:
            self.get_logger().warn(
                'slam_toolbox set_parameters returned no result for '
                'use_scan_matching',
                throttle_duration_sec=2.0,
            )
            return

        result = response.results[0]
        if result.successful:
            self.applied_scan_matching_enabled = expected_value
            self.get_logger().info(
                'slam_toolbox use_scan_matching updated: '
                f'{self.applied_scan_matching_enabled}'
            )
            return

        self.get_logger().warn(
            'slam_toolbox rejected use_scan_matching update: '
            f'value={expected_value}, reason={result.reason}',
            throttle_duration_sec=2.0,
        )

    def log_status(self):
        if self.latest_roll is None or self.latest_pitch is None:
            self.get_logger().info(
                'slam_scan_matching_guard status: waiting for tilt data, '
                f'enable_guard={self.enable_guard}, '
                f'desired={self.desired_scan_matching_enabled}, '
                f'applied={self.applied_scan_matching_enabled}'
            )
            return

        self.get_logger().info(
            'slam_scan_matching_guard status: '
            f'roll={math.degrees(self.latest_roll):.2f}deg, '
            f'pitch={math.degrees(self.latest_pitch):.2f}deg, '
            f'tilted={self.tilted}, '
            f'stable_frames={self.stable_frame_count}, '
            f'desired={self.desired_scan_matching_enabled}, '
            f'applied={self.applied_scan_matching_enabled}, '
            f'pending={self.pending_value}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SlamScanMatchingGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
