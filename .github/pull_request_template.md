## Subsystem
- [ ] `pawn_slam` (SLAM & 3D Raycasting)
- [ ] `pawn_vision` (Human / Crack / Fall-Risk ML)
- [ ] `pawn_autopilot` (Frontier Exploration & Routing)
- [ ] `pawn_gcs` (PySide6 + PyVista Ground Station)
- [ ] `pawn_interfaces` / `pawn_bringup`

## What Changed
- 

## Pre-Merge Verification Checklist
- [ ] `colcon build` succeeds with zero errors inside `pawn_ws/`.
- [ ] **No large binary files** (`.pt`, `.onnx`, `.bag`, `.mcap`, `build/`, `install/`) are included in this commit.
- [ ] All drone topics use relative namespaces (`camera/image_raw`, not `/camera/image_raw`) so multi-drone launch works.
- [ ] ML nodes preserve the original camera `header.stamp` on output messages.
- [ ] Tested against the Ground Station GUI or `mock_publisher_node.py`.
