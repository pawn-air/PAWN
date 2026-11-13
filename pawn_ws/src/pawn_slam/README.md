# pawn_slam

Monocular SLAM for DJI Tello drones, run offboard on the ground station. Work in progress.

```
pawn_slam/
  common/            shared math used by the ROS nodes and offline tools
    transforms.py    SE(3) Pose, quaternions (x, y, z, w), rpy, base_link (FLU) <-> camera_optical_frame
    camera_model.py  pinhole + plumb_bob distortion, loads ROS camera_info YAML
    pose_buffer.py   time-stamped poses, interpolated lookup (no extrapolation)
test/                pytest (colcon test, or plain pytest on a machine without ROS)
```

Frames follow REP-103: `base_link` is FLU (x forward, y left, z up, meters), `camera_optical_frame` is
x right, y down, z forward. `Pose` named `T_a_b` maps points from frame b to frame a.

`tello_attitude_to_rot` assumes aircraft sign conventions for the Tello's roll/pitch/yaw and is **not
verified** yet; `base_to_optical` takes the camera tilt/offset, which are not measured yet.

## Tests

Without ROS (Mac or Linux):

```bash
pip install numpy pyyaml pytest opencv-python   # opencv only for the cross-check test
cd pawn_ws/src/pawn_slam && python -m pytest
```

With ROS 2:

```bash
cd pawn_ws
colcon build --packages-select pawn_slam
colcon test --packages-select pawn_slam && colcon test-result --verbose
```
