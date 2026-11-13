import numpy as np
import pytest

from pawn_slam.common.transforms import (
    Pose, base_to_optical, interpolate, quat_slerp, quat_to_rot, rot_from_rpy, rot_to_quat,
    tello_attitude_to_rot,
)

RNG = np.random.default_rng(0)


def _random_quats(n):
    return np.array([q / np.linalg.norm(q) for q in RNG.normal(size=(n, 4))])


def _is_rotation(R):
    return np.allclose(R.T @ R, np.eye(3)) and np.isclose(np.linalg.det(R), 1.0)


def test_quat_roundtrip_is_canonical():
    for q in _random_quats(200):
        R = quat_to_rot(q)
        assert _is_rotation(R)
        q2 = rot_to_quat(R)
        assert q2[3] >= 0
        assert np.allclose(q2, q if q[3] >= 0 else -q)


@pytest.mark.parametrize("axis", [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0)])
def test_rot_to_quat_at_180_degrees(axis):
    a = np.array(axis, float) / np.linalg.norm(axis)
    q = np.array([*a, 0.0])                                 # 180 deg about a: trace = -1
    assert np.allclose(quat_to_rot(rot_to_quat(quat_to_rot(q))), quat_to_rot(q))


def test_rpy_axes():
    assert np.allclose(rot_from_rpy(0, 0, np.pi / 2) @ [1, 0, 0], [0, 1, 0])   # +yaw turns left
    assert np.allclose(rot_from_rpy(0, np.pi / 2, 0) @ [1, 0, 0], [0, 0, -1])  # +pitch nose down
    assert np.allclose(rot_from_rpy(np.pi / 2, 0, 0) @ [0, 1, 0], [0, 0, 1])   # +roll: left side up
    q = rot_to_quat(rot_from_rpy(0.1, 0.2, 0.3))
    assert np.allclose(q, [0.0342708, 0.1060205, 0.1435722, 0.9833474], atol=1e-6)  # tf2 value


def test_tello_attitude_assumed_signs():
    nose_up = tello_attitude_to_rot(0, 30, 0) @ [1, 0, 0]
    turn_right = tello_attitude_to_rot(0, 0, 90) @ [1, 0, 0]
    assert nose_up[2] > 0 and np.allclose(turn_right, [0, -1, 0])


def test_pose_inverse_compose_apply():
    for q in _random_quats(20):
        T = Pose.from_quat(RNG.normal(size=3), q)
        assert np.allclose((T @ T.inverse()).matrix, np.eye(4))
        assert np.allclose(Pose.from_matrix(T.matrix).matrix, T.matrix)
        p = RNG.normal(size=(5, 3))
        assert np.allclose(T.inverse().apply(T.apply(p)), p)
        assert np.allclose(T.apply(p[0]), (T.matrix @ [*p[0], 1])[:3])


def test_compose_chains_frames():
    T_w_b = Pose(rot_from_rpy(0, 0, np.pi / 2), [1, 0, 0])
    T_b_c = Pose(np.eye(3), [1, 0, 0])                      # 1 m ahead of the body
    assert np.allclose((T_w_b @ T_b_c).t, [1, 1, 0])


def test_base_to_optical():
    T = base_to_optical()
    assert np.allclose(T.R @ [0, 0, 1], [1, 0, 0])          # looks forward
    assert np.allclose(T.R @ [1, 0, 0], [0, -1, 0])         # image right = right
    assert np.allclose(T.R @ [0, 1, 0], [0, 0, -1])         # image down = down
    view = base_to_optical(pitch_down_deg=10).R @ [0, 0, 1]
    assert view[0] > 0 and view[2] < 0                       # forward and a bit down


def test_slerp():
    q0, q1 = [0, 0, 0, 1], rot_to_quat(rot_from_rpy(0, 0, np.pi / 2))
    assert np.allclose(quat_slerp(q0, q1, 0), q0) and np.allclose(quat_slerp(q0, q1, 1), q1)
    assert np.allclose(quat_slerp(q0, q1, 0.5), rot_to_quat(rot_from_rpy(0, 0, np.pi / 4)))
    assert np.allclose(quat_to_rot(quat_slerp(q0, -np.asarray(q1), 0.5)), rot_from_rpy(0, 0, np.pi / 4))


def test_interpolate():
    a, b = Pose(), Pose(rot_from_rpy(0, 0, np.pi / 2), [2, 0, 0])
    m = interpolate(a, b, 0.5)
    assert np.allclose(m.t, [1, 0, 0]) and np.allclose(m.R, rot_from_rpy(0, 0, np.pi / 4))
