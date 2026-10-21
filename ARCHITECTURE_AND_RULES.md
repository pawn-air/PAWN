# PAWN (PİYON) — Architecture, ROS 2 Contracts & Team Rules

This document defines the strict software boundaries, ROS 2 topic/message contracts, and Git workflow for the **PAWN** multi-drone Search & Rescue (SAR) and post-earthquake structural risk analysis system.

---

## 1. System Paradigm: "Ultra-Light Drone, Offboard Brain"
* **Onboard the Drones:** Only capture and stream **Video + Raw IMU** (with hardware timestamps) to the Ground Station. No heavy SLAM or ML runs onboard.
* **On the Ground Station:** Runs all ROS 2 nodes (`pawn_slam`, `pawn_vision`, `pawn_autopilot`) and aggregates everything into the **PySide6 + PyVista Ground Control Station (`pawn_gcs`)**.

---

## 2. Workspace Ownership & Package Boundaries

All ROS 2 code lives inside `pawn_ws/src/`. **Do not put loose Python scripts in the repository root.**

---

## 3. Multi-Drone Namespace & TF Tree Rules

Every drone-specific node and topic **MUST** be namespaced as `/drone_<id>/...` (e.g., `/drone_1`, `/drone_2`). Never hardcode `/camera/image_raw` in your code.

### Coordinate Frames (`tf2` Standard: REP-103)
* All 3D coordinates use **Right-Handed FLU** ($X$ Forward, $Y$ Left, $Z$ Up) in **meters**.
* **TF Tree Hierarchy:**
  * `world` (Global merged SLAM map frame)
    * `└── drone_1/odom` $\rightarrow$ `drone_1/base_link` $\rightarrow$ `drone_1/camera_optical_frame`
    * `└── drone_2/odom` $\rightarrow$ `drone_2/base_link` $\rightarrow$ `drone_2/camera_optical_frame`

---

## 4. The Official ROS 2 Topic & Message Contract

Sub-teams must publish and subscribe **strictly** to these topics so the PySide6 GUI and Autopilot work without rewiring:

### A. Inputs (Drone Receiver $\rightarrow$ ROS 2)
* `/drone_<id>/camera/image_raw` (`sensor_msgs/msg/Image`) — Raw synced video stream.
* `/drone_<id>/imu/data_raw` (`sensor_msgs/msg/Imu`) — Raw synced IMU packets.

### B. ML Team Outputs (`pawn_vision` $\rightarrow$ SLAM Raycaster & GUI)
* `/drone_<id>/vision/detections_2d` (`pawn_interfaces/msg/Detection2D`)
  * Published whenever a **Human (`HUMAN`)**, **Wall Crack (`CRACK`)**, or **Fall Risk (`FALL_RISK`)** is detected.
  * Must copy the exact `header.stamp` from the input image so the SLAM team can look up the exact camera pose at that millisecond!
* `/drone_<id>/vision/hud_annotated` (`sensor_msgs/msg/Image`)
  * Composite image with bounding boxes / crack masks drawn for the PySide6 live video viewport.

### C. SLAM Team Outputs (`pawn_slam` $\rightarrow$ Autopilot & PySide6 3D Viewport)
* `/global_map/pointcloud` (`sensor_msgs/msg/PointCloud2`)
  * Voxel-downsampled 3D map in the `world` frame (published at 2–5 Hz for smooth PyVista rendering).
* `/drone_<id>/slam/pose` (`geometry_msgs/msg/PoseStamped`)
  * Real-time 6-DoF pose of the drone in the `world` frame (20–30 Hz).
* `/hazards/confirmed_3d` (`pawn_interfaces/msg/Hazard3D`)
  * Created by raycasting `Detection2D` pixels against the 3D SLAM map. Deduplicated so the same crack or human isn't spawned 100 times.

### D. Autopilot Outputs (`pawn_autopilot` $\rightarrow$ Drones & PySide6 3D Viewport)
* `/drone_<id>/autopilot/planned_path` (`nav_msgs/msg/Path`) — Planned spline rendered in PyVista.
* `/autopilot/frontiers` (`visualization_msgs/msg/MarkerArray`) — Target frontier spheres and no-fly fall-risk cylinders.

---

## 5. Custom Message Definitions (`pawn_interfaces`)

### `Detection2D.msg`
```text
std_msgs/Header header          # MUST match the timestamp of the source video frame
uint8 drone_id                  # 1, 2, ...
string hazard_type              # Strictly: "HUMAN", "CRACK", or "FALL_RISK"
float32 confidence              # 0.0 to 1.0
float32 severity_score          # 0.0 to 10.0 (used for crack angle/width & fall risk)
float32[4] bbox_xywh            # Normalized [x_center, y_center, w, h] (0.0 to 1.0)
float32[2] raycast_uv           # Normalized [u, v] target pixel to project onto 3D SLAM wall/floor
sensor_msgs/CompressedImage crop_thumbnail

### `Hazard3D.msg`
````test
std_msgs/Header header          # frame_id: "world"
uint32 hazard_id                # Unique deduplicated ID
uint8 drone_id                  # Drone that discovered it
string hazard_type              # Strictly: "HUMAN", "CRACK", or "FALL_RISK"
float32 confidence              # 0.0 to 1.0
float32 severity_score          # 0.0 to 10.0
geometry_msgs/Point position    # 3D [X, Y, Z] coordinates in "world" frame
geometry_msgs/Vector3 normal    # Surface normal of the wall/floor at [X, Y, Z]
sensor_msgs/CompressedImage crop_thumbnail
