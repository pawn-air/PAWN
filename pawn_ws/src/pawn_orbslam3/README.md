# PAWN SLAM v0.1 — ROS 2 Jazzy

## Overview

PAWN runs monocular ORB-SLAM3 **offboard on the ground station**. It estimates camera motion and builds a sparse local map from camera images.

- `pawn_orbslam3`: C++ ROS 2 wrapper publishing pose, diagnostic trajectory and active-map landmarks.
- `pawn_slam`: EuRoC/MP4 playback tools and RViz configuration.

The two launch entry points are `slam_node.launch.py` for integration and `slam_demo.launch.py` for playback. Outputs have arbitrary scale; they are not metric drone poses in a global frame.

## Quick Start

For a machine with ROS 2 Jazzy, Pangolin and the **patched** ORB-SLAM3 already installed. Replace example paths with your own. Start in the PAWN repository root:

```bash
export PAWN_REPO="$PWD"
export ORB_SLAM3_DIR=/path/to/patched/ORB_SLAM3
source /opt/ros/jazzy/setup.bash
# If Pangolin uses a private prefix, also set:
# export CMAKE_PREFIX_PATH="/path/to/pangolin-install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"

cd "$PAWN_REPO/pawn_ws"
colcon build --packages-select pawn_orbslam3 pawn_slam \
  --cmake-args -DORB_SLAM3_DIR="$ORB_SLAM3_DIR"
source install/setup.bash

ros2 launch pawn_orbslam3 slam_demo.launch.py camera_source:=euroc \
  dataset_path:=/path/to/EuRoC/MH01 drone_id:=1 rviz:=true
```

RViz opens while ORB-SLAM3 initializes. Playback starts after approximately 15 seconds; pose and map displays appear once tracking succeeds. Stop the demo with **Ctrl+C** in the launch terminal.

## ROS Interface

`drone_id` selects the `/drone_<id>` node namespace and topic prefix.

| Topic | Message type | Purpose |
|---|---|---|
| `/drone_<id>/camera/image_raw` | `sensor_msgs/msg/Image` | Input; converted to mono8, sensor-data subscription QoS |
| `/drone_<id>/slam/pose` | `geometry_msgs/msg/PoseStamped` | Camera optical pose in the local SLAM frame |
| `/drone_<id>/path` | `nav_msgs/msg/Path` | Optional accumulated diagnostic trajectory |
| `/drone_<id>/slam/map_points` | `sensor_msgs/msg/PointCloud2` | Sparse XYZ landmarks from the current active map |

SLAM outputs use `frame_id: drone_<id>/slam_map` and preserve the input image timestamp. Playback images use `drone_<id>/camera_optical_frame`. Pose is published for each successfully tracked image; path and points every five tracked images. No TF is published.

For example, a mapping consumer for drone 2 subscribes to `/drone_2/slam/pose` and `/drone_2/slam/map_points`. Inspect their headers with:

```bash
ros2 topic echo /drone_2/slam/pose --field header --once
ros2 topic echo /drone_2/slam/map_points --field header --once
```

Treat both as arbitrary-scale local data. Estimate scale and frame alignment before combining drones; `/global_map/pointcloud` belongs to global mapping and is not published here.

## First-Time Installation

Target: **Ubuntu 24.04**, with [ROS 2 Jazzy installed](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). Builds, package tests and bounded EuRoC playback passed on the development host using OpenCV 4.6.0 and Pangolin 0.9.3. The complete clean-machine installation and private-prefix Pangolin recipe below have **not** been validated on a fresh system.

### ROS and system dependencies

Use Bash, starting in the PAWN repository root:

```bash
export PAWN_REPO="$PWD"
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install build-essential cmake git python3-colcon-common-extensions python3-rosdep \
  libopencv-dev libeigen3-dev libboost-serialization-dev libssl-dev \
  libgl1-mesa-dev libegl1-mesa-dev libglew-dev libepoxy-dev libx11-dev libjpeg-dev libpng-dev
# Initialize rosdep once; skip init if this file already exists.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths "$PAWN_REPO/pawn_ws/src/pawn_orbslam3" \
  "$PAWN_REPO/pawn_ws/src/pawn_slam" --ignore-src --rosdistro jazzy -r -y
```

rosdep supplies the ROS/Python dependencies, including cv_bridge, rclpy, launch and RViz. Pangolin and patched ORB-SLAM3 are built separately. Mapping, Gazebo and RealSense are not required by this wrapper.

### Pangolin v0.9.3

The current ORB-SLAM3 core links Pangolin and includes its viewer code even when its own viewer is disabled and RViz is used instead. Pangolin is therefore a build/runtime dependency.

Use new dependency directories; preserve existing working installations. Install Pangolin into a private prefix without changing global library directories. See the [upstream instructions](https://github.com/stevenlovegrove/Pangolin/tree/v0.9.3#building) for optional platform/video dependencies.

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
```

### Pinned ORB-SLAM3 and patch

Use revision `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`, also recorded in `orbslam3_patch/local_orbslam3_revision.txt`. Apply `orbslam3_clean.patch` once, before building: it adds C++17 compatibility, a LoopClosing index fix and the active-map position snapshot API required by the wrapper. Review upstream GPL-3.0 licensing before redistribution.

```bash
export ORB_SLAM3_DIR="$PAWN_DEPS/ORB_SLAM3"
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$ORB_SLAM3_DIR"
# This checkout command applies only to the new dependency repository.
git -C "$ORB_SLAM3_DIR" checkout --detach 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
git -C "$ORB_SLAM3_DIR" apply --check "$PAWN_REPO/orbslam3_patch/orbslam3_clean.patch"
git -C "$ORB_SLAM3_DIR" apply "$PAWN_REPO/orbslam3_patch/orbslam3_clean.patch"

cmake -S "$ORB_SLAM3_DIR/Thirdparty/DBoW2" -B "$ORB_SLAM3_DIR/Thirdparty/DBoW2/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ORB_SLAM3_DIR/Thirdparty/DBoW2/build" -j2
cmake -S "$ORB_SLAM3_DIR/Thirdparty/g2o" -B "$ORB_SLAM3_DIR/Thirdparty/g2o/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ORB_SLAM3_DIR/Thirdparty/g2o/build" -j2
# The core target uses bundled Sophus headers; no separate Sophus install is needed.
cmake -S "$ORB_SLAM3_DIR" -B "$ORB_SLAM3_DIR/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ORB_SLAM3_DIR/build" --target ORB_SLAM3 -j2
tar -xf "$ORB_SLAM3_DIR/Vocabulary/ORBvoc.txt.tar.gz" -C "$ORB_SLAM3_DIR/Vocabulary"
```

This builds the core library without upstream standalone examples. Reduce `-j2` if memory is limited. With these variables still set, run the Quick Start build and source commands, retaining your actual `ORB_SLAM3_DIR` value. Run package tests from `pawn_ws`:

```bash
colcon test --packages-select pawn_orbslam3 pawn_slam
colcon test-result --verbose
```

### New terminals and library paths

After a successful workspace build, restore the environment in each new terminal. These dependency paths match the installation above; adjust them for an existing installation:

```bash
export PAWN_REPO=/path/to/PAWN-repo
export PAWN_DEPS="$HOME/pawn-slam-deps"
export ORB_SLAM3_DIR="$PAWN_DEPS/ORB_SLAM3"
export PANGOLIN_PREFIX="$PAWN_DEPS/pangolin-install"
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH="$PANGOLIN_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
source "$PAWN_REPO/pawn_ws/install/setup.bash"
```

`CMAKE_PREFIX_PATH` (or CMake's `Pangolin_DIR`) locates Pangolin at build time. CMake accepts `ORB_SLAM3_DIR` through the environment or `-DORB_SLAM3_DIR=...`; its fallback is `~/PAWN/ORB_SLAM3`.

Normal installation and optional `--symlink-install` are supported. Installed runtime paths locate the configured ORB-SLAM3 and linked dependencies without manual symlinks or an ORB-specific `LD_LIBRARY_PATH`. Keep those directories available; if they move, reconfigure and rebuild the wrapper. The installation is not a self-contained dependency bundle.

## Launch Options

### Integration: SLAM only

```bash
ros2 launch pawn_orbslam3 slam_node.launch.py drone_id:=2 \
  settings_path:=/path/to/calibrated_camera.yaml
```

This starts only SLAM, awaiting images from the receiver. It does not start playback or RViz.

### Demo: playback and optional RViz

```bash
# EuRoC cam0: source timestamps are preserved.
ros2 launch pawn_orbslam3 slam_demo.launch.py camera_source:=euroc \
  dataset_path:=/path/to/EuRoC/MH01 drone_id:=2 rviz:=true

# MP4: ROS publication timestamps; file FPS, with a 30 Hz fallback.
ros2 launch pawn_orbslam3 slam_demo.launch.py camera_source:=video \
  video_path:=/path/to/video.mp4 settings_path:=/path/to/calibrated_camera.yaml \
  drone_id:=2 rviz:=false
```

The demo waits **approximately 15 seconds before starting playback** to allow ORB-SLAM3 initialization. This is a fixed delay, not a readiness check. Slower systems may need more time; start playback separately after the SLAM ready log if necessary. There is no launch argument to extend this delay.

| Arguments | Applies to | Meaning / default |
|---|---|---|
| `drone_id` | Both | Positive integer; default `1` |
| `orb_slam3_dir` | Both | `ORB_SLAM3_DIR` environment variable, otherwise `~/PAWN/ORB_SLAM3`; keep consistent with the built dependency |
| `settings_path` | Both | Required calibration in integration mode; source-specific default in demo mode |
| `publish_path` | Both | Default `true`; `false` disables trajectory accumulation/publication |
| `image_topic` | Integration | Empty means `/drone_<id>/camera/image_raw` |
| `vocabulary_path` | Integration | Empty means `<orb_slam3_dir>/Vocabulary/ORBvoc.txt` |
| `camera_source` | Demo | `euroc` (default) or `video` |
| `dataset_path` | EuRoC demo | Required directory containing `mav0/cam0/data.csv` and `mav0/cam0/data/` |
| `video_path` | MP4 demo | Required video file |
| `rviz` | Demo | Default `true`; `false` disables RViz |

Calibration must match the actual image resolution, intrinsics and distortion. EuRoC defaults to `<orb_slam3_dir>/Examples/Monocular/EuRoC.yaml`. MP4 defaults to `config/s24_test.yaml`, which is approximate and unsuitable for quantitative geometry; supply actual calibration.

Both launches set namespaces without duplicating the topic prefix. RViz follows the selected drone's topics and local fixed frame. For direct executable use, set both `drone_id` and the matching ROS namespace. Inspect the available arguments:

```bash
ros2 launch pawn_orbslam3 slam_node.launch.py --show-args
ros2 launch pawn_orbslam3 slam_demo.launch.py --show-args
```

## Troubleshooting

| Symptom | Check |
|---|---|
| `install/setup.bash: No such file or directory` | The repository workspace has not been built, its install directory is missing, or the shell is in the wrong directory. Build from `pawn_ws` before sourcing. |
| `Package 'pawn_orbslam3' not found` | Source ROS and this repository's `pawn_ws/install/setup.bash`. Check whether the build completed successfully. |
| Shared library loading error | Source ROS; confirm the configured ORB-SLAM3/Pangolin libraries still exist. If dependency paths changed, rebuild with the correct `ORB_SLAM3_DIR` and Pangolin prefix. Do not add ad hoc symlinks. |
| Missing vocabulary, calibration or dataset | Check `Vocabulary/ORBvoc.txt` extraction, `settings_path`, and the EuRoC directory layout above. Example `/path/to/...` values must be replaced. |
| RViz initially has no trajectory or points | Allow initialization and playback to begin. Outputs require tracking state `OK` (`2`); check image-count/tracking logs, calibration, selected drone and RViz fixed frame `drone_<id>/slam_map`. |

## Current Limitations

- **Monocular camera poses have arbitrary scale.** Coordinates use ORB-SLAM3's local optical convention, not metric FLU drone body poses. The pose topic does not yet satisfy the metric `world` semantics in `ARCHITECTURE_AND_RULES.md`.
- The map is sparse and local. It is not a dense reconstruction or metric global map, and does not directly establish free space for occupancy mapping.
- IMU fusion, camera-to-body calibration, coordinate conversion, a TF tree, multi-drone map merging and Hazard3D are future integration work.
- Tracking resets can create a new local map and coordinate jumps while the diagnostic path retains older poses. The path grows while enabled; disable it for long runs. Do not run the offline `trajectory_publisher` alongside live SLAM for the same drone.
- Snapshot safety under concurrent atlas/map resets has not been independently certified. Datasets and videos are not included.
