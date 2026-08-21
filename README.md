<p align="center">
  <h1 align="center">🎯 RCIA Vision 2025</h1>
  <p align="center">A ROS 2 pipeline for armor detection, target tracking, pose solving, and visualization.</p>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-Humble-blue.svg" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/C%2B%2B-17%20%7C%2020-blue.svg" alt="C++ 17 and 20">
  <img src="https://img.shields.io/badge/OpenCV-4.x-green.svg" alt="OpenCV 4">
  <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="MIT License">
</p>

<p align="center">
  <a href="README.md">English</a> | <a href="README.zh.md">简体中文</a>
</p>

## Overview

RCIA Vision 2025 is a ROS 2 RoboMaster vision stack for real-time armor detection, IPPE/PnP pose estimation, Ceres refinement, EKF tracking, latency compensation, and gimbal control with camera, serial, and Foxglove integration.

The stack is organized around explicit ROS topic and component boundaries between perception, estimation, control, hardware integration, and simulation. It also supports Foxglove observability and an optional Unreal Engine 5 bridge for closed-loop validation.

Core technologies include ROS 2 Humble, C++17/C++20, Python 3, OpenCV, Eigen, Ceres Solver, ONNX Runtime, tf2, Foxglove Bridge, and Unreal Engine 5 integration.

## Features

- Real-time armor detection, contour processing, and target classification.
- IPPE/PnP pose initialization with corner refinement and reprojection-based geometry checks.
- Reduced-degree-of-freedom Ceres bundle adjustment for multi-frame yaw refinement.
- Nine-state EKF tracking of vehicle translation, height, yaw, angular velocity, and armor radius.
- Layered latency compensation covering pipeline time, projectile flight, prediction, and gimbal response.
- Target selection, ballistic compensation, gimbal command generation, and serial output.
- Foxglove observability, watchdog recovery, trajectory simulation, and UE5 closed-loop validation.

## Architecture

```text
 Camera Callback ---> Color Separation ---> Light-Bar Pairing
                              |
                              v
                    Corner Refinement / Digit Classification
                              |
                              v
                       IPPE/PnP Pose Initialization
                              |
                              v
                    Reduced-DOF BA ---> tf2 / odom Transform
                              |
                              v
                 9-State EKF ---> Target Selection ---> Ballistic Compensation
                              |
                              v
                    Gimbal Control ---> Serial Command Output
                              |
                              +------> Foxglove / UE5 Bridge

 Heartbeat Publishers ---> Watchdog / Recovery ---> Launch Session
```

## Core Modules

| Module | Path | Responsibility | Key input / output |
| --- | --- | --- | --- |
| Interfaces | `src/rcia_vision_interfaces` | Defines ROS 2 messages shared by the pipeline. | Image, state, pose, heartbeat, and command messages. |
| Detector | `src/rcia_vision_detector` | Performs color processing, light-bar pairing, corner refinement, digit classification, and IPPE/PnP initialization. | Camera frames and robot state / armor observations and initial pose. |
| Solver | `src/rcia_math_solver` | Applies Ceres multi-frame yaw refinement and exposes solver results. | Detection messages / refined pose and aiming data. |
| Tracker | `src/rcia_vision_tracker` | Tracks the vehicle with a nine-state EKF, selects targets, compensates motion, and generates gimbal commands. | Pose, odometry, and target state / control messages and heartbeat. |
| Drivers | `src/rcia_sensor_driver` | Connects camera and serial hardware to ROS 2. | Vendor camera frames and serial packets. |
| Bringup | `src/rcia_bringup` | Starts components and loads runtime parameters. | Launch arguments / composed ROS 2 system. |
| Guard | `src/rcia_vision_guard` | Monitors heartbeats and recovers runtime windows. | Heartbeat messages / restart actions. |
| Utilities | `src/rcia_vision_utils` | Provides trajectory simulation and supporting tools. | Trajectory messages / plots and diagnostics. |
| Visualization | `src/ros2_foxglove_bridge` | Bridges ROS 2 topics to Foxglove for inspection and runtime diagnosis. | ROS 2 topics / WebSocket visualization stream. |

## Project Structure

The tree below presents the ROS 2 packages and their primary source units in a flattened view. Build metadata and physical source paths remain unchanged.

```text
RCIA-Vision-2025/
├── rcia_vision_interfaces/               # Shared ROS 2 interfaces
├── rcia_vision_detector/                 # Detection, classification, and PnP
│   ├── classify_model/                   # ONNX classification models
│   ├── angle_solver                      # Camera-angle calculation
│   ├── armor_detector                    # Armor contour and light-bar detection
│   ├── armor_identify                    # Armor target identification
│   ├── corner_correct                    # Detection corner refinement
│   ├── debug                             # Detection diagnostics and overlays
│   ├── image_subscriber                   # Camera image subscription
│   ├── pattern_classify                   # Digit classification
│   └── pnp_solver                        # IPPE/PnP pose estimation
├── rcia_math_solver/                     # Pose refinement and bundle adjustment
│   ├── ba_solver                         # Ceres bundle adjustment
│   └── math_solver_node                  # Solver ROS 2 node
├── rcia_vision_tracker/                  # Tracking and gimbal control
│   ├── gimbal_control                     # Gimbal command generation
│   ├── guard_dog_publish                 # Tracker heartbeat publishing
│   ├── spinTop_predictor                 # Target motion prediction
│   ├── spinTop_tracker                   # Target state tracking
│   ├── trajectory_compensator            # Flight and latency compensation
│   └── vision_tracker_node               # Tracker ROS 2 node
├── rcia_sensor_driver/                   # Hardware integration packages
│   ├── rcia_camera_driver/               # Camera driver package
│   │   └── camera_node                    # Camera ROS 2 node
│   └── rcia_serial_driver/               # Gimbal and serial driver package
│       ├── protocol                       # Serial protocol definitions
│       ├── infantry_protocol              # Infantry protocol implementation
│       ├── serial_driver_node             # Serial ROS 2 node
│       └── uart_transporter               # UART transport layer
├── rcia_bringup/                         # Runtime orchestration package
│   ├── config/node_params/                # Node parameter files
│   └── launch/                            # System launch descriptions
├── rcia_vision_guard/                    # Watchdog and recovery node
│   └── guard_dog                         # Guard implementation
├── rcia_robot_description/               # Robot model package
│   ├── meshes/                            # Robot mesh resources
│   └── urdf/                              # URDF and Xacro descriptions
├── rcia_vision_utils/                    # Trajectory tools and simulation
│   └── trajectory_simulation.py           # Trajectory simulation utility
├── ros2_foxglove_bridge/                 # Foxglove visualization bridge
│   ├── generic_client                     # Generic ROS 2 client
│   ├── message_definition_cache           # Message definition cache
│   ├── param_utils                        # Parameter utilities
│   ├── parameter_interface                # Parameter interface
│   ├── ros2_foxglove_bridge               # Foxglove bridge implementation
│   ├── ros2_foxglove_bridge_node          # Foxglove bridge node
│   ├── smoke_test                          # Bridge smoke test
│   └── utils_test                          # Bridge utility tests
├── scripts/
│   ├── build/                            # Reproducible build entry points
│   └── run/                              # Runtime and visualization launchers
├── docs/                                 # Practical command and operations docs
│   ├── notes/                            # Quick command reference
│   └── operations/                       # tmux and runtime procedures
├── requirements.txt                      # Optional Python tooling dependencies
├── .gitignore                            # Build, data, model, and IDE exclusions
├── LICENSE                               # MIT license
├── README.md                             # English documentation
└── README.zh.md                          # Chinese documentation
```

## Installation

### Prerequisites

- Ubuntu 22.04 and ROS 2 Humble.
- `colcon`, `rosdep`, C++17/C++20 toolchain, OpenCV, Eigen, and Ceres Solver.
- ONNX Runtime for classification and a compatible vendor camera SDK for hardware runs.
- Foxglove Studio and `foxglove_bridge` for visualization.

### Setup

```bash
git clone https://github.com/AcmeX/RCIA-Vision-2025.git
cd RCIA-Vision-2025
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r requirements.txt
```

Set `ONNXRUNTIME_ROOT` when ONNX Runtime is installed outside `/opt/onnxruntime`. Set `ONEAPI_ROOT` and `IPP_ROOT_DIR` when using the optional Intel IPP build path. Camera SDK libraries must be installed separately.

## Usage

Build the supported core packages:

```bash
source /opt/ros/humble/setup.bash
bash scripts/build/build_workspace.sh
source install/setup.bash
```

Start the composed perception and control pipeline:

```bash
ros2 launch rcia_bringup bringup.launch.py
```

Start the operational helpers in separate terminals when needed:

```bash
bash scripts/run/run_guard.sh
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

For command groups, package-level builds, and tmux session management, see [docs/notes/quick_command.md](docs/notes/quick_command.md) and [docs/operations/tmux.md](docs/operations/tmux.md).

## Engineering Results

The following values are project-level records from specific hardware, scenes, and test scripts. They are not universal benchmarks; reproductions should report camera configuration, target distance, timestamps, transport path, and statistical protocol together with the result.

| Metric | Recorded result |
| --- | --- |
| Pose synchronization error | Below 1.5 degrees |
| Transport latency | Below 5 ms in the measured link |
| Virtual-to-real ballistic impact error | Below 5 cm in the project test setup |
| Validation efficiency | More than 3x the previous workflow |

The engineering gain comes from separating the perception, solver, tracking, and control boundaries, while using simulation and replay to turn intermittent failures into reproducible test cases.

## Technical Notes

The design rationale, mathematical derivations, and validation records are documented in the companion article: [RCIA Vision Overview](https://acmex-cosmos.github.io/blog/rcia-vision-overview/).

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for the full text.

## Contact

For questions, issues, or contributions:

- **GitHub Issues**: [https://github.com/AcmeX/RCIA-Vision-2025/issues](https://github.com/AcmeX/RCIA-Vision-2025/issues)
- **Email**: AcmeX@foxmail.com

---