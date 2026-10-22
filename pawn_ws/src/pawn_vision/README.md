# PAWN Vision (`pawn_vision`)

This ROS 2 package provides the complete multi-modal Search & Rescue (SAR) Perception, Hazard Detection, and Real-Time Risk Assessment engine for the **PAWN** multi-drone autonomous exploration system.

---

## 1. Subsystem Architecture

The package integrates **6 specialized lightweight YOLOv11 deep learning models** and a deterministic **INSARAG/AFAD Risk Assessment & Triage Engine** operating under a 30–40 FPS **Frame Multiplexing Scheduler**:

| Module | Task | Model Architecture | Metric / Benchmark |
| :--- | :--- | :--- | :--- |
| **Module 1** | Human & Pet Detection + 17-Keypoint Pose | `yolo11n.pt` + `yolo11n-pose.pt` | Distinguishes Fallen Victims (Critical P1) vs. Standing Normal (P2). |
| **Module 2** | Flame & Smoke Detection | `fire_smoke_yolo.pt` (D-Fire dataset) | Dual verification: Flame core + Canny tongue edge roughness. |
| **Module 3** | ISO 7010 & GHS Hazard Signs | `hazard_signs_yolo11n.pt` | **mAP50: 99.0%** across 19 official international danger templates. |
| **Module 4** | Structural Cracks & Heavy Damage (v4) | `crack_seg_yolo.pt` + `heavy_structural_yolo11n.pt` | **mAP50: 89.6%** (Spalling: 98.2%, Exposed Rebar: 99.5%, Brick: 70.9%). Optimized for both 25px–75px localized damage and heavy collapses. |
| **Module 5** | Multi-Modal Risk Engine & Triage | Deterministic MCDM / Expert System | **0.012 ms latency**. Computes 4D risk vector, compound synergies, and INSARAG P1–P5 codes. |

---

## 2. Topic & ROS 2 Contract (`ARCHITECTURE_AND_RULES.md`)

* **Subscribed Topics:**
  * `/drone_<id>/camera/image_raw` (`sensor_msgs/msg/Image`) — Synced raw video stream from drone receiver.
* **Published Topics:**
  * `/drone_<id>/vision/hud_annotated` (`sensor_msgs/msg/Image`) — Composite viewport with bounding boxes, segmentation masks, and live Risk HUD for PySide6 GCS. Preserves input `header.stamp`.
  * `/drone_<id>/vision/detections_2d` (`pawn_interfaces/msg/Detection2D`) — 2D bounding boxes and `raycast_uv` pins for the SLAM raycasting engine to spawn 3D hazard pins.

---

## 3. Quick Start: Setting Up Model Weights

Model weights are managed via the official weights script. Run from the package directory:

```bash
cd pawn_ws/src/pawn_vision/weights
bash download_weights.sh
```

* **Local Machine:** If trained weights exist locally in `Desktop/Machine Learning/models`, they are linked/copied with zero download time.
* **Remote Machines:** Downloads official verified models and validates their SHA-256 checksums automatically.
* **Force Update:** Use `bash download_weights.sh --force` to refresh models to the latest release.

---

## 4. Standalone Testing (Without ROS 2)

You can test the vision engine standalone using your webcam, video files, or test images:

### Live Webcam Stream (FaceTime / USB Camera):
```bash
python3 -m pawn_vision.sar_video_pipeline --source 0 --display
```

### Video File Processing (Saves Annotated Output):
```bash
python3 -m pawn_vision.sar_video_pipeline --source test_video.mp4 --output outputs/annotated_output.mp4
```

### Single Photo Full Analysis:
```bash
python3 -m pawn_vision.sar_master_pipeline test_image.jpg
```

---

## 5. ROS 2 Launch

Inside your ROS 2 workspace:

```bash
cd pawn_ws
colcon build --packages-select pawn_vision
source install/setup.bash

# Launch vision node for Drone 1:
ros2 launch pawn_vision pawn_vision.launch.py drone_id:=1

# Launch vision node for Drone 2:
ros2 launch pawn_vision pawn_vision.launch.py drone_id:=2
```
