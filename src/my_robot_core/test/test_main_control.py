import math

import pytest
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time

from my_robot_core.main_control import (
    ARM_BASE_FRAME_ID,
    ARM_OFFSET_X,
    ARM_OFFSET_Y,
    CAMERA_LINK_FRAME_ID,
    CAMERA_OPTICAL_FRAME_ID,
    CAM_SEARCH,
    IK_APPROACH_HEIGHT_MM,
    IK_LINK_L0_MM,
    IK_LINK_L1_MM,
    IK_LINK_L2_MM,
    IK_LINK_L3_MM,
    MainControlNode,
)
from jibot1_driver.kinematics import Kinematics


class _ClockNow:
    def to_msg(self):
        return Time()


class _Clock:
    def now(self):
        return _ClockNow()


class _RecordingBroadcaster:
    def __init__(self):
        self.messages = []

    def sendTransform(self, msg):
        self.messages.append(msg)


def _make_uninitialized_main_control():
    return object.__new__(MainControlNode)


def test_nav2_status_name_reports_action_status_labels():
    assert MainControlNode._nav2_status_name(
        GoalStatus.STATUS_SUCCEEDED
    ) == 'SUCCEEDED'
    assert MainControlNode._nav2_status_name(
        GoalStatus.STATUS_ABORTED
    ) == 'ABORTED'
    assert MainControlNode._nav2_status_name(999) == '999'


def test_camera_tf_broadcast_uses_mount_link_then_optical_frame():
    node = _make_uninitialized_main_control()
    node.current_posture = 'SEARCH'
    node.tf_search = CAM_SEARCH
    node.tf_observe = CAM_SEARCH
    node.camera_frame_id = CAMERA_OPTICAL_FRAME_ID
    node.arm_offset_x = ARM_OFFSET_X
    node.arm_offset_y = ARM_OFFSET_Y
    node.tf_broadcaster = _RecordingBroadcaster()
    node.get_clock = lambda: _Clock()

    node.broadcast_dynamic_tf()

    assert len(node.tf_broadcaster.messages) == 1
    mount_tf, optical_tf, arm_tf = node.tf_broadcaster.messages[0]
    assert mount_tf.header.frame_id == 'base_footprint'
    assert mount_tf.child_frame_id == CAMERA_LINK_FRAME_ID
    assert mount_tf.transform.translation.x == CAM_SEARCH['x']
    assert mount_tf.transform.translation.y == CAM_SEARCH['y']
    assert mount_tf.transform.translation.z == CAM_SEARCH['z']

    assert optical_tf.header.frame_id == CAMERA_LINK_FRAME_ID
    assert optical_tf.child_frame_id == CAMERA_OPTICAL_FRAME_ID
    assert optical_tf.transform.translation.x == 0.0
    assert optical_tf.transform.translation.y == 0.0
    assert optical_tf.transform.translation.z == 0.0
    assert optical_tf.transform.rotation.x == pytest.approx(-0.5)
    assert optical_tf.transform.rotation.y == pytest.approx(0.5)
    assert optical_tf.transform.rotation.z == pytest.approx(-0.5)
    assert optical_tf.transform.rotation.w == pytest.approx(0.5)
    norm = sum([
        optical_tf.transform.rotation.x ** 2,
        optical_tf.transform.rotation.y ** 2,
        optical_tf.transform.rotation.z ** 2,
        optical_tf.transform.rotation.w ** 2,
    ])
    assert math.isclose(norm, 1.0)

    assert arm_tf.header.frame_id == 'base_footprint'
    assert arm_tf.child_frame_id == ARM_BASE_FRAME_ID
    assert arm_tf.transform.translation.x == ARM_OFFSET_X
    assert arm_tf.transform.translation.y == ARM_OFFSET_Y
    assert arm_tf.transform.translation.z == 0.0


def test_arm_done_status_marks_pick_complete():
    node = _make_uninitialized_main_control()
    node._pick_done = False
    node._pick_failed = False
    node._last_arm_status = ''
    node.get_logger = lambda: type(
        'Logger',
        (),
        {
            'info': lambda *_args, **_kwargs: None,
            'error': lambda *_args, **_kwargs: None,
        },
    )()

    msg = type('Msg', (), {'data': 'PICK_ONLY:DONE'})()
    node.arm_status_cb(msg)

    assert node._pick_done is True
    assert node._pick_failed is False
    assert node._last_arm_status == 'PICK_ONLY:DONE'


def test_arm_failure_status_marks_pick_failed():
    node = _make_uninitialized_main_control()
    node._pick_done = True
    node._pick_failed = False
    node._last_arm_status = ''
    node.get_logger = lambda: type(
        'Logger',
        (),
        {
            'info': lambda *_args, **_kwargs: None,
            'error': lambda *_args, **_kwargs: None,
        },
    )()

    msg = type('Msg', (), {'data': 'IK_FAILED:100,420,35'})()
    node.arm_status_cb(msg)

    assert node._pick_done is False
    assert node._pick_failed is True
    assert node._last_arm_status == 'IK_FAILED:100,420,35'


def test_pick_reach_check_rejects_far_ball_target():
    node = _make_uninitialized_main_control()
    node.ik_reach_check_enabled = True
    node.ik_approach_height = IK_APPROACH_HEIGHT_MM
    node._pick_kinematics = Kinematics(
        l0=IK_LINK_L0_MM,
        l1=IK_LINK_L1_MM,
        l2=IK_LINK_L2_MM,
        l3=IK_LINK_L3_MM,
    )

    assert node._is_pick_target_reachable(0.0, 220.0, 35.0) is True
    assert node._is_pick_target_reachable(103.4, 422.0, 35.0) is False


def test_obstacle_ahead_uses_configured_scan_front_angle_for_reversed_lidar():
    node = _make_uninitialized_main_control()
    node.obstacle_scan_angle = math.radians(30.0)
    node.scan_front_angle_offset = math.pi
    node.obstacle_stop_dist = 0.2
    node.scan_angle_min = -math.pi
    node.scan_angle_increment = math.radians(1.0)
    node.scan_ranges = [math.inf] * 361

    # Laser angle 0 is physically behind the robot when the lidar is mounted
    # with yaw=pi, so it must not trigger the front obstacle recovery.
    node.scan_ranges[180] = 0.1
    assert node.is_obstacle_ahead() is False

    # +/-pi is the robot front in the current reversed-lidar installation.
    node.scan_ranges[180] = math.inf
    node.scan_ranges[0] = 0.1
    assert node.is_obstacle_ahead() is True
    assert node.get_front_obstacle_angle() == pytest.approx(0.0)


def test_obstacle_ahead_still_supports_zero_degree_front_scan():
    node = _make_uninitialized_main_control()
    node.obstacle_scan_angle = math.radians(30.0)
    node.scan_front_angle_offset = 0.0
    node.obstacle_stop_dist = 0.2
    node.scan_angle_min = -math.pi
    node.scan_angle_increment = math.radians(1.0)
    node.scan_ranges = [math.inf] * 361

    node.scan_ranges[180] = 0.1
    assert node.is_obstacle_ahead() is True


def test_reversed_lidar_obstacle_side_drives_lateral_escape_direction():
    node = _make_uninitialized_main_control()
    node.obstacle_scan_angle = math.radians(30.0)
    node.scan_front_angle_offset = math.pi
    node.obstacle_stop_dist = 0.2
    node.scan_angle_min = -math.pi
    node.scan_angle_increment = math.radians(1.0)
    node.scan_ranges = [math.inf] * 361
    node.recovery_side_step_speed = 0.08
    node.recovery_side_step_time = 0.8
    node.obstacle_side_deadband = math.radians(5.0)
    node.recovery_center_escape_direction = 'right'

    # With the current reversed lidar, physical front-left is near laser -170 deg.
    node.scan_ranges[10] = 0.1
    obstacle_angle = node.get_front_obstacle_angle()
    assert obstacle_angle == pytest.approx(math.radians(10.0))
    assert node._select_recovery_lateral_speed(obstacle_angle) == pytest.approx(-0.08)

    # Physical front-right is near laser +170 deg.
    node.scan_ranges[10] = math.inf
    node.scan_ranges[350] = 0.1
    obstacle_angle = node.get_front_obstacle_angle()
    assert obstacle_angle == pytest.approx(math.radians(-10.0))
    assert node._select_recovery_lateral_speed(obstacle_angle) == pytest.approx(0.08)


def test_center_obstacle_uses_configured_default_escape_direction():
    node = _make_uninitialized_main_control()
    node.recovery_side_step_speed = 0.08
    node.recovery_side_step_time = 0.8
    node.obstacle_side_deadband = math.radians(5.0)

    node.recovery_center_escape_direction = 'right'
    assert node._select_recovery_lateral_speed(0.0) == pytest.approx(-0.08)

    node.recovery_center_escape_direction = 'left'
    assert node._select_recovery_lateral_speed(0.0) == pytest.approx(0.08)
