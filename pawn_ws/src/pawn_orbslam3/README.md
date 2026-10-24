# PAWN monocular SLAM (ROS 2 Jazzy)

`pawn_orbslam3` wraps ORB-SLAM3; `pawn_slam` supplies EuRoC/MP4 playback and RViz configuration. There are exactly two launch entry points: `slam_node.launch.py` for integration and `slam_demo.launch.py` for playback with optional RViz.

## Install and build

Use Ubuntu 24.04, ROS 2 Jazzy, rosdep, colcon and a C++17 toolchain. Install ROS dependencies from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths pawn_ws/src/pawn_orbslam3 pawn_ws/src/pawn_slam --ignore-src -r -y
```

Install Pangolin separately and build ORB-SLAM3 with its bundled DBoW2/g2o and upstream prerequisites (OpenCV, Eigen, Sophus). Pangolin must be discoverable through `CMAKE_PREFIX_PATH` or `Pangolin_DIR`. Review upstream GPL-3.0 licensing before redistribution.

Use a **separate dependency checkout** at revision `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`, also recorded in `orbslam3_patch/local_orbslam3_revision.txt`. Before building that checkout, apply `orbslam3_patch/orbslam3_clean.patch` from this repository:

```bash
# Run in the separate ORB-SLAM3 dependency checkout; replace the patch path.
git apply --check /path/to/PAWN-repo/orbslam3_patch/orbslam3_clean.patch
git apply /path/to/PAWN-repo/orbslam3_patch/orbslam3_clean.patch
# Follow upstream build instructions; extract Vocabulary/ORBvoc.txt if needed.
```

The patch adds C++17 compatibility, a LoopClosing index type fix and an active-map position snapshot API. Do not reapply it to an already patched working installation.

```bash
# From this repository's pawn_ws:
export ORB_SLAM3_DIR=/path/to/separate/ORB_SLAM3
colcon build --packages-select pawn_orbslam3 pawn_slam --symlink-install \
  --cmake-args -DORB_SLAM3_DIR="$ORB_SLAM3_DIR"
source install/setup.bash
```

CMake accepts `-DORB_SLAM3_DIR=...` or the environment variable; its fallback is `~/PAWN/ORB_SLAM3`. Launch also accepts `orb_slam3_dir:=...`. That directory must contain built shared libraries and `Vocabulary/ORBvoc.txt`.

## Launch

```bash
# Integration: only the SLAM node, awaiting the receiver's image stream.
ros2 launch pawn_orbslam3 slam_node.launch.py drone_id:=2 \
  settings_path:=/path/to/calibrated_camera.yaml

# Playback: EuRoC cam0, source timestamps preserved.
ros2 launch pawn_orbslam3 slam_demo.launch.py camera_source:=euroc \
  dataset_path:=/path/to/EuRoC/MH01 drone_id:=2

# Playback: MP4, ROS receipt timestamps, file FPS (30 Hz fallback).
ros2 launch pawn_orbslam3 slam_demo.launch.py camera_source:=video \
  video_path:=/path/to/video.mp4 settings_path:=/path/to/calibrated_camera.yaml \
  drone_id:=2 rviz:=false
```

Calibration must match the input resolution, intrinsics and distortion. EuRoC defaults to the dependency checkout's `Examples/Monocular/EuRoC.yaml`. The MP4 default `config/s24_test.yaml` is approximate and unsuitable for quantitative geometry; supply actual calibration.

Both launches accept `drone_id` (positive integer), `orb_slam3_dir`, `settings_path`, and `publish_path:=false` to disable diagnostic trajectory accumulation/publication. Integration additionally accepts `vocabulary_path` and `image_topic` overrides. Demo accepts `camera_source:=euroc|video`, `dataset_path`, `video_path`, and `rviz:=true|false`. It starts playback after a 15-second delay; vocabulary loading on a slower machine may require starting playback separately after the ready log. Use `--show-args` for argument defaults. Nodes launched by either entry point use namespace `/drone_<id>`; absolute topic defaults prevent duplicated prefixes. Direct executable use should supply both `drone_id` and the matching ROS namespace.

## Interface and expected output

| Topic | Type | Meaning |
|---|---|---|
| `/drone_<id>/camera/image_raw` | `sensor_msgs/msg/Image` | Input, converted to mono8; sensor-data QoS |
| `/drone_<id>/slam/pose` | `geometry_msgs/msg/PoseStamped` | Camera optical pose in the local SLAM frame |
| `/drone_<id>/path` | `nav_msgs/msg/Path` | Optional accumulated trajectory |
| `/drone_<id>/slam/map_points` | `sensor_msgs/msg/PointCloud2` | Current active map's sparse XYZ landmarks |

Pose, path and points carry `frame_id: drone_<id>/slam_map` and the source image timestamp. Playback images carry `drone_<id>/camera_optical_frame`. No TF is published. RViz selects the requested drone's topics and local fixed frame. Logs report image count, tracking-state changes and tracked frames. Outputs require tracking state `OK` (2); pose is emitted per tracked image, path/points every five tracked images. The offline `trajectory_publisher` helper accepts `trajectory_file` and `drone_id`; never run it alongside live SLAM for the same drone.

## Integration limits

This is **monocular, not visual-inertial**: positions and sparse landmarks have arbitrary scale and use ORB-SLAM3's local optical-coordinate convention. They are not metric FLU drone body poses in global `world`. Publishing the agreed pose topic does not yet satisfy that final semantic contract in `ARCHITECTURE_AND_RULES.md`.

Metric scale, camera-to-body calibration, coordinate conversion, IMU fusion, TF, global registration/map merging and Hazard3D raycasting are future integration responsibilities. Global mapping owns `/global_map/pointcloud`; this module never publishes it. Do not concatenate independent drone maps or infer metric occupancy/free space directly from these sparse points.

An ORB tracking reset can create a new active map: its coordinates may jump while the diagnostic path still contains older poses. The path grows while enabled; disable it for long runs. The snapshot accessor locks map updates and copies finite positions, but atlas/map lifetime safety under concurrent resets has not been independently certified. No datasets, binaries or learned models are shipped.
