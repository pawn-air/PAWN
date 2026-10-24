# PAWN monocular SLAM (ROS 2 Jazzy)

`pawn_orbslam3` wraps ORB-SLAM3; `pawn_slam` supplies EuRoC/MP4 playback and RViz configuration. There are exactly two launch entry points: `slam_node.launch.py` for integration and `slam_demo.launch.py` for playback with optional RViz.

## Install and build

Target environment: Ubuntu 24.04 with [ROS 2 Jazzy installed](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). The existing host used OpenCV 4.6.0 and Pangolin 0.9.3. The wrapper and patched ORB core were built on that host; a clean-machine installation of the entire recipe below has **not** been tested. Review ORB-SLAM3's GPL-3.0 license before redistribution.

### ROS and system prerequisites

From the PAWN repository root, in Bash:

```bash
export PAWN_REPO="$PWD"
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install build-essential cmake git python3-colcon-common-extensions python3-rosdep \
  libopencv-dev libeigen3-dev libboost-serialization-dev libssl-dev \
  libgl1-mesa-dev libegl1-mesa-dev libglew-dev libepoxy-dev libx11-dev libjpeg-dev libpng-dev
# First use only: skip init if /etc/ros/rosdep/sources.list.d/20-default.list already exists.
sudo rosdep init
rosdep update
rosdep install --from-paths "$PAWN_REPO/pawn_ws/src/pawn_orbslam3" \
  "$PAWN_REPO/pawn_ws/src/pawn_slam" --ignore-src --rosdistro jazzy -r -y
```

rosdep supplies ROS/Python dependencies including cv_bridge, rclpy, launch and RViz. Pangolin and the patched ORB-SLAM3 core are built separately below. No IMU, mapping, Gazebo or RealSense dependency is needed by this wrapper.

### Pangolin and ORB-SLAM3

Use **new, separate directories**, preserving any working ORB-SLAM3 installation. The [Pangolin v0.9.3 upstream instructions](https://github.com/stevenlovegrove/Pangolin/tree/v0.9.3#building) describe optional platform/video dependencies. The commands below install into a private prefix, without global library changes:

```bash
export PAWN_DEPS="$HOME/pawn-slam-deps"
export PANGOLIN_PREFIX="$PAWN_DEPS/pangolin-install"
mkdir -p "$PAWN_DEPS"
git clone --recursive --branch v0.9.3 https://github.com/stevenlovegrove/Pangolin.git "$PAWN_DEPS/Pangolin"
cmake -S "$PAWN_DEPS/Pangolin" -B "$PAWN_DEPS/Pangolin/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PANGOLIN_PREFIX" \
  -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_RPATH='$ORIGIN' \
  -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_TESTS=OFF -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build "$PAWN_DEPS/Pangolin/build" -j2
cmake --install "$PAWN_DEPS/Pangolin/build"
export CMAKE_PREFIX_PATH="$PANGOLIN_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"

export ORB_SLAM3_DIR="$PAWN_DEPS/ORB_SLAM3"
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$ORB_SLAM3_DIR"
git -C "$ORB_SLAM3_DIR" checkout --detach 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
git -C "$ORB_SLAM3_DIR" apply --check "$PAWN_REPO/orbslam3_patch/orbslam3_clean.patch"
git -C "$ORB_SLAM3_DIR" apply "$PAWN_REPO/orbslam3_patch/orbslam3_clean.patch"

cmake -S "$ORB_SLAM3_DIR/Thirdparty/DBoW2" -B "$ORB_SLAM3_DIR/Thirdparty/DBoW2/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ORB_SLAM3_DIR/Thirdparty/DBoW2/build" -j2
cmake -S "$ORB_SLAM3_DIR/Thirdparty/g2o" -B "$ORB_SLAM3_DIR/Thirdparty/g2o/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ORB_SLAM3_DIR/Thirdparty/g2o/build" -j2
# Bundled Sophus is used through its headers; no separate install is needed for this core target.
cmake -S "$ORB_SLAM3_DIR" -B "$ORB_SLAM3_DIR/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ORB_SLAM3_DIR/build" --target ORB_SLAM3 -j2
tar -xf "$ORB_SLAM3_DIR/Vocabulary/ORBvoc.txt.tar.gz" -C "$ORB_SLAM3_DIR/Vocabulary"
```

The revision is also recorded in `orbslam3_patch/local_orbslam3_revision.txt`. The patch supplies C++17 compatibility, a LoopClosing index fix and the active-map snapshot API. Apply it once, before building. The core target avoids building upstream standalone examples. Reduce build parallelism if memory is limited.

### Build the two ROS packages

Keep the variables above in the same shell, or restore them in each new terminal:

```bash
source /opt/ros/jazzy/setup.bash
# Set ORB_SLAM3_DIR to your patched checkout and CMAKE_PREFIX_PATH to your Pangolin prefix.
cd "$PAWN_REPO/pawn_ws"
colcon build --packages-select pawn_orbslam3 pawn_slam \
  --cmake-args -DORB_SLAM3_DIR="$ORB_SLAM3_DIR"
source install/setup.bash
colcon test --packages-select pawn_orbslam3 pawn_slam
colcon test-result --verbose
```

Normal installation and optional `--symlink-install` are supported. The installed executable records library search paths derived from `ORB_SLAM3_DIR` and linked dependencies (including Pangolin); manual symlinks or an ORB-specific `LD_LIBRARY_PATH` export are unnecessary. Keep those dependency directories available. If they move, reconfigure and rebuild the wrapper against their new locations; this is not a self-contained redistributable bundle.

CMake accepts `-DORB_SLAM3_DIR=...` or its environment variable; the fallback is `~/PAWN/ORB_SLAM3`. Launch accepts `orb_slam3_dir:=...` too: keep it consistent with the built dependency. The checkout must contain `lib/libORB_SLAM3.so`, the DBoW2/g2o shared libraries and `Vocabulary/ORBvoc.txt`. `CMAKE_PREFIX_PATH` (or CMake's `Pangolin_DIR`) is needed to discover a nonstandard Pangolin installation at build time. Source both ROS and this workspace's `install/setup.bash` before launch.

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
