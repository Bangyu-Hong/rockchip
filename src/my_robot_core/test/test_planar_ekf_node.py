import math

import numpy as np
import pytest

from my_robot_core.planar_ekf_node import (
    STATE_SIZE,
    STATE_VX,
    STATE_VY,
    STATE_WZ,
    STATE_X,
    STATE_Y,
    STATE_YAW,
    PlanarEkf,
    axis_delta,
    normalize_angle,
)


def test_planar_ekf_predicts_body_velocity_in_odom_frame():
    ekf = PlanarEkf([0.0] * STATE_SIZE)
    ekf.initialize(
        0.0,
        x=0.0,
        y=0.0,
        yaw=math.pi / 2.0,
        vx=1.0,
        vy=0.0,
        wz=0.0,
    )

    ekf.predict_to(1.0, max_dt=1.0)

    assert ekf.state[STATE_X] == pytest.approx(0.0, abs=1e-6)
    assert ekf.state[STATE_Y] == pytest.approx(1.0, abs=1e-6)


def test_planar_ekf_wraps_yaw_innovation():
    ekf = PlanarEkf([0.0] * STATE_SIZE)
    ekf.initialize(0.0, yaw=math.radians(179.0))

    h = np.zeros((1, STATE_SIZE), dtype=float)
    h[0, STATE_YAW] = 1.0
    ekf.update([math.radians(-179.0)], h, np.diag([0.01]), angle_measurement_rows=[0])

    assert abs(normalize_angle(ekf.state[STATE_YAW] - math.pi)) < math.radians(2.0)


def test_planar_ekf_updates_wheel_translation_and_imu_yaw_separately():
    ekf = PlanarEkf([0.01] * STATE_SIZE)
    ekf.initialize(0.0, x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0)

    wheel_h = np.zeros((4, STATE_SIZE), dtype=float)
    wheel_h[0, STATE_X] = 1.0
    wheel_h[1, STATE_Y] = 1.0
    wheel_h[2, STATE_VX] = 1.0
    wheel_h[3, STATE_VY] = 1.0
    ekf.update([1.0, 0.0, 0.4, 0.0], wheel_h, np.diag([0.01] * 4))

    imu_h = np.zeros((2, STATE_SIZE), dtype=float)
    imu_h[0, STATE_YAW] = 1.0
    imu_h[1, STATE_WZ] = 1.0
    ekf.update([0.2, 0.1], imu_h, np.diag([0.01, 0.01]), angle_measurement_rows=[0])

    assert ekf.state[STATE_X] > 0.5
    assert ekf.state[STATE_VX] > 0.2
    assert ekf.state[STATE_YAW] > 0.1
    assert ekf.state[STATE_WZ] > 0.05


def test_visual_axis_delta_maps_camera_optical_to_base_planar_axes():
    delta_xyz = np.array([0.1, 0.0, 0.3], dtype=float)

    assert axis_delta(delta_xyz, 'z') == pytest.approx(0.3)
    assert axis_delta(delta_xyz, '-x') == pytest.approx(-0.1)


def test_high_visual_twist_variance_keeps_low_ekf_gain():
    low_weight = PlanarEkf([0.0] * STATE_SIZE)
    high_weight = PlanarEkf([0.0] * STATE_SIZE)
    low_weight.initialize(0.0, vx=0.0)
    high_weight.initialize(0.0, vx=0.0)
    low_weight.covariance = np.eye(STATE_SIZE, dtype=float) * 0.01
    high_weight.covariance = np.eye(STATE_SIZE, dtype=float) * 0.01

    h = np.zeros((1, STATE_SIZE), dtype=float)
    h[0, STATE_VX] = 1.0
    low_weight.update([1.0], h, np.diag([0.35 ** 2]))
    high_weight.update([1.0], h, np.diag([0.05 ** 2]))

    assert low_weight.state[STATE_VX] < high_weight.state[STATE_VX]
    assert low_weight.state[STATE_VX] < 0.1
