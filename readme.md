# Johnny 5 ROS2 Workspace

Complete ROS 2 (Jazzy) workspace for the Johnny 5 hobby robot, including URDF/Xacro model, launch files, and helper scripts.

## Workspace Layout

```
johnny_5_ws/                    # Complete ROS2 workspace
├── readme.md                   # This file
├── src/
│   └── johnny_5_description/   # Robot description package
│       ├── urdf/               # johnny_5.xacro
│       ├── meshes/             # STL/DAE files (empty for now)
│       ├── launch/             # view_robot.launch.py
│       ├── include/            # C++ headers (empty)
│       ├── src/                # C++ source (empty)
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── LICENSE
├── build/                      # Colcon build artifacts
├── install/                    # Installation files
└── log/                        # Build logs
```

## Quick Start

```bash
# Clone the complete workspace
cd ~
git clone https://github.com/mattjenson/johnny_5_ws.git
cd johnny_5_ws

# Build the workspace
colcon build
source install/setup.bash

# Desktop run (has DISPLAY)
ros2 launch johnny_5_description view_robot.launch.py

# Headless / SSH run (no DISPLAY)
DISPLAY= ros2 launch johnny_5_description view_robot.launch.py
```

## Runtime Dependencies

| Package | Purpose |
|---------|---------|
| `robot_state_publisher` | Publishes TF tree from URDF |
| `xacro` | Converts Xacro → URDF at launch time |
| `joint_state_publisher_gui` | Slider GUI for joint testing (desktop) |
| `joint_state_publisher` | CLI fallback for headless targets |
| `rviz2` | Visualization |
| `launch`, `launch_ros` | Python launch framework |

Install via apt:

```bash
sudo apt update && \
  sudo apt install ros-jazzy-robot-state-publisher \
                   ros-jazzy-xacro \
                   ros-jazzy-joint-state-publisher-gui \
                   ros-jazzy-joint-state-publisher \
                   ros-jazzy-rviz2
```

## Display-aware Launch Behavior

- **If `$DISPLAY` is set**, `view_robot.launch.py` starts:
  - `joint_state_publisher_gui` (sliders)
  - `rviz2` with the robot model
- **If no display is detected** (e.g. SSH to the robot), it automatically falls back to:
  - `joint_state_publisher` (CLI)
  - No RViz, preventing Qt/X11 crashes

## Feature Status

| Feature | Status |
|---------|--------|
| Xacro skeleton loads in RViz | ✅ Working |
| GUI ⇄ CLI joint publisher fallback | ✅ Working |
| Visual & collision meshes | 🚧 Not added yet |
| Joint limits & dynamics | 🚧 WIP |
| Unit tests (pytest/launch) | ❌ Not started |

Feel free to open PRs or issues as you contribute!

## Roadmap / Planned Package Structure

Below is the high-level directory and package plan we'll follow as the project grows. Each bullet will become its own package (or group of packages) when we reach that stage.

```
johnny_5_description/          # URDF/Xacro, meshes, SRDF
johnny_5_interfaces/           # custom msg/srv/action definitions

# ── Sensor drivers (one per sensor) ───────────────────────────
johnny_5_cam_driver/           # RGB/Depth camera
johnny_5_mic_driver/           # Microphone array / audio capture
johnny_5_lidar_driver/         # 2-D/3-D distance scanner
johnny_5_imu_compass_driver/   # IMU + magnetometer
johnny_5_gps_driver/           # GNSS receiver

# ── ros2_control hardware plugins ─────────────────────────────
johnny_5_base_hw/
johnny_5_head_hw/
johnny_5_body_rotation_hw/
johnny_5_arm_hw/

# ── Corresponding controllers (reuse std when possible) ───────
johnny_5_base_controller/
johnny_5_head_controller/
johnny_5_body_rotation_controller/
johnny_5_arm_controller/

# ── Higher-level stacks ───────────────────────────────────────
johnny_5_navigation/           # Nav2 params + launch
johnny_5_perception/           # Vision / audio AI nodes
johnny_5_behaviors/            # Behavior Trees or state machines

# ── Integration & deployment ──────────────────────────────────
johnny_5_bringup/              # Default launch, composes nodes
johnny_5_simulation/           # Ignition Gazebo model & launch
johnny_5_tests/                # System & integration tests

# (optional) Unit-specific calibration
johnny_5_1_calibration/        # Camera intrinsics, IMU offsets
johnny_5_1_bringup/            # Thin launch pointing to *_calibration
```

**Status**: we're currently focused on `johnny_5_description`. As each milestone above is tackled we'll create a new package, document its purpose in this README, and update the roadmap table.

## License

Licensed under the Apache License 2.0. See LICENSE for details.
