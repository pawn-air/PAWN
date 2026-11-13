import threading

import numpy as np

from pawn_slam.common.pose_buffer import PoseBuffer
from pawn_slam.common.transforms import Pose, rot_from_rpy


def _yaw_pose(x, yaw):
    return Pose(rot_from_rpy(0, 0, yaw), [x, 0, 0])


def test_interpolates_between_samples():
    b = PoseBuffer()
    b.add(10.0, _yaw_pose(0, 0))
    b.add(11.0, _yaw_pose(2, np.pi / 2))
    p = b.lookup(10.25)
    assert np.allclose(p.t, [0.5, 0, 0]) and np.allclose(p.R, rot_from_rpy(0, 0, np.pi / 8))


def test_exact_hits_and_no_extrapolation():
    b = PoseBuffer()
    assert b.lookup(1.0) is None and b.span() is None
    a, c = _yaw_pose(0, 0), _yaw_pose(1, 0)
    b.add(1.0, a)
    b.add(2.0, c)
    assert b.lookup(1.0) is a and b.lookup(2.0) is c
    assert b.lookup(0.999) is None and b.lookup(2.001) is None
    assert b.span() == (1.0, 2.0) and len(b) == 2


def test_out_of_order_and_replace():
    b = PoseBuffer()
    for t in (3.0, 1.0, 2.0):
        b.add(t, _yaw_pose(t, 0))
    assert np.allclose(b.lookup(1.5).t, [1.5, 0, 0])
    b.add(2.0, _yaw_pose(5, 0))
    assert len(b) == 3 and np.allclose(b.lookup(2.0).t, [5, 0, 0])


def test_max_gap():
    b = PoseBuffer(max_gap=0.5)
    b.add(0.0, _yaw_pose(0, 0))
    b.add(0.4, _yaw_pose(1, 0))
    b.add(2.0, _yaw_pose(2, 0))                  # tracking lost for 1.6 s
    assert b.lookup(0.2) is not None and b.lookup(1.0) is None
    assert b.lookup(2.0) is not None              # samples themselves are still returned


def test_max_age_drops_old_samples():
    b = PoseBuffer(max_age=1.0)
    for i in range(30):
        b.add(i * 0.1, _yaw_pose(i, 0))
    lo, hi = b.span()
    assert np.isclose(hi, 2.9) and hi - lo <= 1.0 + 1e-9
    assert b.lookup(1.0) is None


def test_threads():
    b = PoseBuffer(max_age=5.0)

    def writer(k):
        for i in range(500):
            b.add(k + i * 1e-3, _yaw_pose(i, 0))

    ts = [threading.Thread(target=writer, args=(k,)) for k in range(4)]
    for t in ts:
        t.start()
    for _ in range(500):
        b.lookup(1.25)
    for t in ts:
        t.join()
    times = b._t
    assert times == sorted(times) and len(times) == len(set(times)) == len(b._p)
