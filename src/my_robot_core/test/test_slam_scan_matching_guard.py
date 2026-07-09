import math

from rcl_interfaces.msg import ParameterType

from my_robot_core.slam_scan_matching_guard import (
    DEFAULT_ENABLE_SLAM_SCAN_MATCHING_GUARD,
    SlamScanMatchingGuard,
)


def _make_uninitialized_guard():
    return object.__new__(SlamScanMatchingGuard)


def _configure_guard_thresholds(node):
    node.drop_roll_threshold = math.radians(2.0)
    node.drop_pitch_threshold = math.radians(2.0)
    node.resume_roll_threshold = math.radians(1.0)
    node.resume_pitch_threshold = math.radians(1.0)
    node.stable_frames_required = 3
    node.level_scan_matching_enabled = True
    node.tilted_scan_matching_enabled = False
    node.tilted = False
    node.stable_frame_count = 0
    node.desired_scan_matching_enabled = True


def test_tilt_state_disables_scan_matching_until_stably_level():
    node = _make_uninitialized_guard()
    _configure_guard_thresholds(node)

    assert node._update_tilt_state(math.radians(2.1), 0.0)
    assert node.tilted
    assert not node.desired_scan_matching_enabled

    assert not node._update_tilt_state(math.radians(1.5), 0.0)
    assert node.tilted
    assert node.stable_frame_count == 0

    assert not node._update_tilt_state(math.radians(0.9), math.radians(0.8))
    assert node.tilted
    assert node.stable_frame_count == 1

    assert not node._update_tilt_state(math.radians(0.8), math.radians(0.7))
    assert node.tilted
    assert node.stable_frame_count == 2

    assert node._update_tilt_state(math.radians(0.7), math.radians(0.6))
    assert not node.tilted
    assert node.desired_scan_matching_enabled
    assert node.stable_frame_count == 0


def test_scan_matching_request_sets_slam_toolbox_bool_parameter():
    node = _make_uninitialized_guard()

    request = node._make_scan_matching_request(False)

    assert len(request.parameters) == 1
    parameter = request.parameters[0]
    assert parameter.name == 'use_scan_matching'
    assert parameter.value.type == ParameterType.PARAMETER_BOOL
    assert not parameter.value.bool_value


def test_slam_scan_matching_guard_default_is_disabled():
    assert DEFAULT_ENABLE_SLAM_SCAN_MATCHING_GUARD is False
