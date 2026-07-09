import math
import time

import pytest
from sensor_msgs.msg import LaserScan

from my_robot_core.laser_filter_node import (
    DEFAULT_TILTED_SLOPE_SCAN_RANGE_M,
    LaserFilterNode,
    TILTED_SCAN_MODE_ALL_MAPS,
    TILTED_SCAN_MODE_SAFETY_ONLY,
)


def _make_uninitialized_filter():
    return object.__new__(LaserFilterNode)


class _RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_tilted_safety_scan_keeps_only_near_finite_ranges():
    node = _make_uninitialized_filter()
    node.tilted_safety_max_range = DEFAULT_TILTED_SLOPE_SCAN_RANGE_M
    below_limit = DEFAULT_TILTED_SLOPE_SCAN_RANGE_M - 0.01
    above_limit = DEFAULT_TILTED_SLOPE_SCAN_RANGE_M + 0.01

    scan = LaserScan()
    scan.range_min = 0.05
    scan.range_max = 8.0
    scan.ranges = [
        0.04,
        0.05,
        below_limit,
        DEFAULT_TILTED_SLOPE_SCAN_RANGE_M,
        above_limit,
        math.inf,
        math.nan,
        1.0,
    ]
    scan.intensities = [1.0, 2.0]

    safety_scan = node._make_tilted_safety_scan(scan)

    assert math.isnan(safety_scan.ranges[0])
    assert safety_scan.ranges[1] == pytest.approx(0.05)
    assert safety_scan.ranges[2] == pytest.approx(below_limit)
    assert safety_scan.ranges[3] == pytest.approx(DEFAULT_TILTED_SLOPE_SCAN_RANGE_M)
    assert math.isnan(safety_scan.ranges[4])
    assert math.isnan(safety_scan.ranges[5])
    assert math.isnan(safety_scan.ranges[6])
    assert math.isnan(safety_scan.ranges[7])
    assert list(safety_scan.intensities) == [1.0, 2.0]


def test_tilted_trusted_scan_mode_controls_filtered_output():
    node = _make_uninitialized_filter()
    node.tilted_safety_max_range = DEFAULT_TILTED_SLOPE_SCAN_RANGE_M
    node.tilted_scan_mode = TILTED_SCAN_MODE_ALL_MAPS
    node.scan_pub = _RecordingPublisher()
    above_limit = DEFAULT_TILTED_SLOPE_SCAN_RANGE_M + 0.01

    scan = LaserScan()
    scan.range_min = 0.05
    scan.range_max = 8.0
    scan.ranges = [0.2, DEFAULT_TILTED_SLOPE_SCAN_RANGE_M, above_limit, math.inf]

    assert node._publish_tilted_trusted_scan(scan)
    assert len(node.scan_pub.messages) == 1
    trusted_scan = node.scan_pub.messages[0]
    assert trusted_scan.ranges[0] == pytest.approx(0.2)
    assert trusted_scan.ranges[1] == pytest.approx(DEFAULT_TILTED_SLOPE_SCAN_RANGE_M)
    assert math.isnan(trusted_scan.ranges[2])
    assert math.isnan(trusted_scan.ranges[3])

    node.tilted_scan_mode = TILTED_SCAN_MODE_SAFETY_ONLY
    assert not node._publish_tilted_trusted_scan(scan)
    assert len(node.scan_pub.messages) == 1


def test_default_tilted_slope_scan_range_is_operator_tunable_constant():
    assert DEFAULT_TILTED_SLOPE_SCAN_RANGE_M == pytest.approx(0.70)


def test_startup_bootstrap_stops_after_level_scan_or_limits():
    node = _make_uninitialized_filter()
    node.startup_tilt_bootstrap_enabled = True
    node.has_published_level_scan = False
    node.startup_bootstrap_frames_used = 0
    node.startup_tilt_bootstrap_max_frames = 3
    node.startup_tilt_bootstrap_max_duration_sec = 2.0
    node.startup_tilt_bootstrap_max_angle = math.radians(5.0)
    node.startup_time = time.monotonic()

    assert node._startup_bootstrap_allowed(
        math.radians(2.1),
        math.radians(4.9),
    )

    node.has_published_level_scan = True
    assert not node._startup_bootstrap_allowed(
        math.radians(2.1),
        math.radians(4.9),
    )

    node.has_published_level_scan = False
    node.startup_bootstrap_frames_used = 3
    assert not node._startup_bootstrap_allowed(
        math.radians(2.1),
        math.radians(4.9),
    )

    node.startup_bootstrap_frames_used = 0
    assert not node._startup_bootstrap_allowed(
        math.radians(2.1),
        math.radians(5.1),
    )

    node.startup_time = time.monotonic() - 3.0
    assert not node._startup_bootstrap_allowed(
        math.radians(2.1),
        math.radians(4.9),
    )
