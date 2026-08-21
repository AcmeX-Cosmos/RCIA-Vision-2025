# Sensor Driver Packages

This directory contains the ROS 2 hardware integration packages used by the vision pipeline.

- `rcia_camera_driver`: camera component based on the vendor SDK.
- `rcia_serial_driver`: serial transport, protocol parsing, and ROS message bridge.

Both packages expose ROS 2 components and are launched by `rcia_bringup`. Hardware SDK paths are configured in the corresponding CMake files and must be installed on the target system before building.
