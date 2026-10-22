# PAWN (PİYON) — Project & Ground Control Station Context for LLMs

> **Usage Instruction for AI Assistants:** This document serves as the single source of truth for the **PAWN (PİYON)** senior capstone project (`https://github.com/pawn-air/PAWN`). Read these architectural constraints, ROS 2 topic contracts, file trees, and environment rules carefully before generating code or architectural advice.


## 1. Project Overview & Core Paradigm
* **Project Name:** PAWN (PİYON) — Autonomous Multi-Drone Search & Rescue (SAR) and Post-Earthquake Structural Risk Assessment System.
* **Core Paradigm ("Ultra-Light Drone, Offboard Brain"):**
  * **Onboard the Drones:** Ultra-lightweight payload. The drones do **not** run heavy onboard compute. Each drone only streams **Synced Video + Raw IMU** packets back to the Ground Control Station (GCS).
  * **On the Ground Station (Offboard Compute):** Runs all heavy computation locally as **ROS 2 nodes**, including Visual-Inertial SLAM (VI-SLAM), 3D map merging, multiple Machine Learning (ML) computer vision pipelines, autonomous frontier exploration (Autopilot), and the desktop GUI.

## 2. Repository & ROS 2 Workspace Structure
All ROS 2 packages live inside `pawn_ws/src/` in the monorepo (`pawn-air/PAWN`), keeping the root project website isolated.

```text
PAWN/
├── docs/ or website files          # Project information website (GitHub Pages)
├── ARCHITECTURE_AND_RULES.md       # Team rules & ROS 2 API contracts
├── LLM_CONTEXT.md                  # This context file
├── .gitignore                      # Blocks build/, install/, log/, *.pt, *.onnx, *.mcap, .venv/
└── pawn_ws/
    └── src/
        ├── pawn_interfaces/        # Custom CMake ROS 2 .msg and .srv package
        │   └── msg/
        │       ├── Detection2D.msg
        │       └── Hazard3D.msg
        ├── pawn_slam/              # VI-SLAM wrappers, map merger, 2D->3D raycaster node
        ├── pawn_vision/            # ML nodes (human, crack, fall-risk) + git-ignored weights/
        ├── pawn_autopilot/         # Multi-drone frontier exploration & risk-aware routing
        ├── pawn_bringup/           # Launch files & mock data publishers
        └── pawn_gcs/               # PySide6 + PyVista Ground Control Station Desktop GUI
            ├── package.xml
            ├── setup.py
            └── pawn_gcs/
                ├── __init__.py
                ├── app.py                      # Entry point, Qt6 Fusion dark palette
                ├── main_window.py              # Master QMainWindow layout & signal wiring
                ├── data_models.py              # Pure Python dataclasses (HazardEvent, etc.)
                ├── workers/
                │   ├── __init__.py
                │   ├── mock_sim_worker.py      # Standalone QThread simulator (runs without ROS)
                │   └── ros_bridge_worker.py    # QThread spinning rclpy & emitting Qt Signals
                └── widgets/
                    ├── __init__.py
                    ├── fleet_mission_bar.py    # Top bar: mode status, layer toggles, conf slider, E-Stop
                    ├── slam_viewport_3d.py     # Embedded PyVista (QtInteractor) 3D map & drone actors
                    ├── drone_hud_card.py       # Dual drone video viewport with QPainter vector HUD
                    └── triage_inspector.py     # P0/P1/P2 hazard list + detail card + 3D camera fly-to

```

---

## 3. Ground Control Station (`pawn_gcs`) Architecture & Tech Stack

* **GUI Framework:** **PySide6 (Qt6)** + **PyVista (`pyvistaqt.QtInteractor`)** + **NumPy** + **`opencv-python-headless`**.
* **Strict Threading Rule (`QThread` $\rightarrow$ `Signal` $\rightarrow$ `Slot`):**
* Never call PySide6 or PyVista UI methods directly inside a ROS 2 subscriber callback.
* Background workers (`MockMissionWorker` or `RosBridgeWorker`) inherit from `QThread`, unpack data into pure Python/NumPy objects (`HazardEvent`, `np.ndarray`, `dict`), and emit thread-safe `PySide6.QtCore.Signal` events to slots in `PawnGroundStation`.


* **In-Place PyVista Rendering (Performance Rule):**
* Never call `plotter.add_mesh()` repeatedly in a high-frequency loop for moving drones or point clouds (causes GPU memory leaks).
* Pre-allocate `pv.PolyData` / `pv.Sphere` actors during initialization and mutate `.points` in-place, followed by `self.plotter.update()`.


* **UI Layout & Key Features:**
1. **Top Bar (`FleetMissionBar`):** Global mission state badge, visibility checkboxes (`Survivors P0`, `Cracks P2`, `Frontiers`), live ML Confidence Threshold slider (`0.20`–`0.95`), theme toggle (`☀️ Light Mode` / `🌙 Dark Mode`), and `HOLD ALL DRONES` emergency stop.
2. **Left Panel (`SlamViewport3D`):** 3D `QtInteractor` rendering the merged point cloud, color-coded drone spheres (`#38bdf8` Cyan for Drone 1, `#c084fc` Purple for Drone 2), planned trajectory splines, cyan frontier target spheres, color-coded 3D wall hazard pins, and translucent red floor drop-zone discs (`pv.Disc` at $Z=0$) beneath overhead `FALL_RISK` detections. Includes camera preset buttons (`3D Isometric`, `2D Floorplan` using parallel orthographic projection, and `Reset View`), with theme-adaptive background and grid colors.
3. **Top-Right Panel (`DroneHudCard` x2):** Side-by-side live video feeds for Drone 1 and Drone 2 with integrated manual flight override. Renders onboard H.264 camera frames beneath `QPainter` vector overlays (bounding boxes, crack skeletons, telemetry header bar, and vertical structural risk meter). Includes `AUTO` / `MANUAL` toggle, `Takeoff` / `Land` flight buttons, battery percentage (`🔋 88%`), and keyboard manual flight control (`WASD` for pitch/roll, `Arrows` for throttle/yaw, `Space` for flight toggle, `1`/`2` to switch drone focus).
4. **Bottom-Right Panel (`TriageInspector`):** Master-detail view of discovered hazards (`[P0 SURVIVOR]`, `[P1 FALL RISK]`, `[P2 CRACK]`). Clicking any item updates the right-hand inspector card (coordinates, severity score, structural metrics), allows dismissing false positives (which removes the 3D actor from PyVista), and triggers `self.slam_view.plotter.fly_to(xyz)` to smoothly focus the 3D camera on that defect. Fully themed for dark and light contrast.


## 4. ROS 2 Naming, Coordinate Frames & Message Contracts

* **Multi-Drone Namespacing:** All drone-specific topics and frames are namespaced under `/drone_<id>/...` (e.g., `/drone_1`, `/drone_2`).
* **Coordinate Convention (REP-103):** Right-handed **FLU** ($X$ Forward, $Y$ Left, $Z$ Up) in meters. Global frame is `world`.
* **Timestamp Rule:** ML nodes in `pawn_vision` **must** copy the original camera frame's `header.stamp` onto `Detection2D` messages so the SLAM raycaster can query `tf2` for the exact historical camera pose when the frame was captured.
* **Key Topics:**
* `/drone_<id>/camera/image_raw` (`sensor_msgs/msg/Image`)
* `/drone_<id>/imu/data_raw` (`sensor_msgs/msg/Imu`)
* `/drone_<id>/vision/detections_2d` (`pawn_interfaces/msg/Detection2D`) — 2D ML output with normalized `bbox_xywh`, `raycast_uv`, `hazard_type` (`"HUMAN"`, `"CRACK"`, `"FALL_RISK"`), `confidence`, and `severity_score` (`0.0`–`10.0`).
* `/global_map/pointcloud` (`sensor_msgs/msg/PointCloud2`) — Downsampled 3D map in `world` frame.
* `/drone_<id>/slam/pose` (`geometry_msgs/msg/PoseStamped`)
* `/drone_<id>/autopilot/planned_path` (`nav_msgs/msg/Path`)
* `/hazards/confirmed_3d` (`pawn_interfaces/msg/Hazard3D`) — Deduplicated 3D hazard position (`geometry_msgs/Point`) and wall surface normal (`geometry_msgs/Vector3`) in `world` frame.

## 5. Development Environment & Known Technical Gotchas (CRITICAL FOR CODE GENERATION)

1. **Strict Qt6 Scoped Enums:** Always write full scoped enums in PySide6 code so it never throws `AttributeError` across different Qt6 bindings:
* Use `QPalette.ColorRole.Window`, `QPalette.ColorRole.WindowText`, `QPalette.ColorRole.Base`, etc. (NOT `QPalette.Window`).
* Use `Qt.Orientation.Horizontal` (NOT `Qt.Horizontal`).
* Use `Qt.ItemDataRole.UserRole` (NOT `Qt.UserRole`).
* Use `QPainter.RenderHint.Antialiasing` and `QFont.Weight.Bold`.


2. **macOS Apple Silicon (M4 Pro) + Python + VTK/PySide6 Environment Rules:**
* Always use a **Python 3.12** virtual environment (`/opt/homebrew/opt/python@3.12/bin/python3.12 -m venv .venv`) rather than Python 3.14, because `vtk` and `PySide6` have official prebuilt wheels on Python 3.12 (which also matches Ubuntu 24.04 + ROS 2 Jazzy).
* Always install `opencv-python-headless` (NEVER `opencv-python`), and ensure Homebrew's global Qt (`/opt/homebrew/Cellar/qtbase` or `qt@5`) does not hijack `QT_QPA_PLATFORM_PLUGIN_PATH`.
* In `pawn_gcs/app.py`, always set `os.environ["QT_API"] = "pyside6"` and instantiate `QApplication(sys.argv)` **before** importing `PawnGroundStation` / `pyvista` so PySide6's Cocoa platform plugin initializes cleanly before VTK loads.



```

```