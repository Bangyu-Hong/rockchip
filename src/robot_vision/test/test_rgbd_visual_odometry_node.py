import numpy as np
import pytest

from robot_vision.rgbd_visual_odometry_node import (
    DEFAULT_MAX_DEPTH_M,
    DEFAULT_MIN_DEPTH_M,
    pad_depth_to_color_size,
    pixel_to_camera_xyz,
    quaternion_from_rotation_matrix,
    sample_depth_at,
)


def test_sample_depth_at_filters_reliable_range_and_uses_median():
    depth = np.array([
        [0, 100, 500],
        [700, 800, 4000],
        [0, 900, 1000],
    ], dtype=np.uint16)

    depth_m = sample_depth_at(
        depth,
        x=1,
        y=1,
        radius=1,
        depth_scale=1000.0,
        min_depth_m=DEFAULT_MIN_DEPTH_M,
        max_depth_m=DEFAULT_MAX_DEPTH_M,
    )

    assert depth_m == pytest.approx(0.8)


def test_pad_depth_to_color_size_centers_short_depth_image():
    depth = np.ones((2, 4), dtype=np.uint16)

    padded = pad_depth_to_color_size(depth, target_h=4, target_w=4)

    assert padded.shape == (4, 4)
    assert np.all(padded[1:3, :] == 1)
    assert np.all(padded[0, :] == 0)
    assert np.all(padded[3, :] == 0)


def test_pixel_to_camera_xyz_uses_camera_intrinsics():
    point = pixel_to_camera_xyz(
        x=420.0,
        y=240.0,
        depth_m=1.0,
        fx=500.0,
        fy=500.0,
        cx=320.0,
        cy=240.0,
    )

    assert point[0] == pytest.approx(0.2)
    assert point[1] == pytest.approx(0.0)
    assert point[2] == pytest.approx(1.0)


def test_quaternion_from_rotation_matrix_identity():
    quat = quaternion_from_rotation_matrix(np.eye(3))

    assert quat.w == pytest.approx(1.0)
    assert quat.x == pytest.approx(0.0)
    assert quat.y == pytest.approx(0.0)
    assert quat.z == pytest.approx(0.0)
