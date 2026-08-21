# RCIA Vision 2025 Repository Layout

```text
RCIA-Vision-2025/
├── src/
│   ├── rcia_vision_interfaces/   # shared ROS 2 interfaces
│   ├── rcia_vision_detector/     # armor detection and classification
│   ├── rcia_math_solver/         # pose refinement and ballistic solving
│   ├── rcia_vision_tracker/      # target tracking and gimbal control
│   ├── rcia_sensor_driver/       # camera and serial integration
│   ├── rcia_bringup/             # launch and parameter orchestration
│   ├── rcia_vision_guard/        # watchdog and recovery
│   ├── rcia_robot_description/   # URDF, Xacro, and meshes
│   ├── rcia_vision_utils/        # trajectory and simulation utilities
│   └── ros2_foxglove_bridge/     # Foxglove visualization bridge
├── scripts/
│   ├── build/                    # reproducible build entry points
│   └── run/                      # runtime launch helpers
├── docs/
│   ├── notes/                    # command references
│   └── operations/               # tmux and runtime procedures
├── requirements.txt              # optional Python tooling dependencies
├── LICENSE                       # MIT license
├── README.md                     # English project entry point
└── README.zh.md                  # Chinese project entry point
```

Package responsibilities:

- `rcia_vision_interfaces`: custom message definitions
- `rcia_vision_utils`: trajectory visualization helper
- `rcia_vision_detector`: armor detection and classification
- `rcia_math_solver`: pose and ballistic solver
- `rcia_vision_guard`: watchdog and recovery process
- `rcia_vision_tracker`: target tracking and control logic
- `rcia_bringup`: launch and parameter orchestration
- `rcia_sensor_driver`: camera and serial hardware integration
