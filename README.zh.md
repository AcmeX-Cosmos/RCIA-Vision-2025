<p align="center">
  <h1 align="center">🎯 RCIA Vision 2025</h1>
  <p align="center">基于 ROS 2 的装甲目标检测、跟踪、位姿解算与可视化流水线。</p>
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

## 项目简介

RCIA Vision 2025 是一个面向实时机器人视觉的模块化 ROS 2 工作区，集成相机与串口输入、装甲板检测与分类、PnP 位姿估计、束调整优化、目标跟踪、云台控制、守护恢复和 Foxglove 可视化。

项目适用于需要将感知、估计、控制和硬件接入通过清晰 ROS 话题与组件边界组织起来的机器人平台。

核心技术包括 ROS 2 Humble、C++17/C++20、Python 3、OpenCV、Eigen、Ceres Solver、ONNX Runtime、tf2 和 Foxglove Bridge。

## 核心特性

- 实时装甲板检测、轮廓处理与目标分类。
- 基于 PnP 的位姿估计，以及重投影误差和几何辅助计算。
- 束调整与弹道解算组件，用于位姿优化和瞄准计算。
- 目标跟踪、运动补偿与云台控制指令生成。
- 通过 ROS 2 组件接入相机和串口驱动。
- 提供 launch 编排、守护恢复、弹道仿真和 Foxglove 支持。

## 系统架构

```text
 相机驱动                       串口驱动
     |                              |
     v                              v
 图像话题  --->  装甲板检测 / 分类
                         |
                         v
                   ROS 2 接口消息
                      /          \\
                     v            v
              PnP / BA 解算    目标跟踪
                     |            |
                     +-----> 云台控制
                                  |
                                  v
                             串口控制输出

 心跳发布器 ---> 守护与恢复 ---> 启动会话
                       |
                       v
                Foxglove Bridge / 工具
```

## 核心模块

| 模块 | 路径 | 职责 | 主要输入 / 输出 |
| --- | --- | --- | --- |
| 接口 | `src/rcia_vision_interfaces` | 定义流水线共享的 ROS 2 消息。 | 图像、状态、位姿、心跳和控制消息。 |
| 检测 | `src/rcia_vision_detector` | 检测装甲目标、分类并估计初始位姿。 | 相机帧、机器人状态 / 装甲观测。 |
| 解算 | `src/rcia_math_solver` | 优化位姿并计算弹道相关结果。 | 检测消息 / 优化位姿和瞄准数据。 |
| 跟踪 | `src/rcia_vision_tracker` | 跟踪目标并进行运动补偿。 | 位姿、里程计 / 目标状态和控制指令。 |
| 驱动 | `src/rcia_sensor_driver` | 将相机和串口硬件接入 ROS 2。 | 厂商相机帧、串口数据包。 |
| 启动 | `src/rcia_bringup` | 启动组件并加载运行参数。 | launch 参数 / 组合式 ROS 2 系统。 |
| 守护 | `src/rcia_vision_guard` | 监控心跳并恢复运行窗口。 | 心跳消息 / 重启动作。 |
| 工具 | `src/rcia_vision_utils` | 提供弹道仿真和辅助工具。 | 轨迹消息 / 图表和诊断结果。 |

## 项目结构

下方结构图采用扁平化源码视图，直接展示 ROS 2 功能包及其主要源码单元；实际构建元数据和源码物理路径保持不变。

```text
RCIA-Vision-2025/
├── rcia_vision_interfaces/               # ROS 2 共享接口
├── rcia_vision_detector/                 # 检测、分类与 PnP 解算
│   ├── classify_model/                   # ONNX 分类模型
│   ├── angle_solver                      # 相机角度计算
│   ├── armor_detector                    # 装甲板轮廓与灯条检测
│   ├── armor_identify                    # 装甲目标识别
│   ├── corner_correct                    # 检测角点修正
│   ├── debug                             # 检测诊断与图像叠加
│   ├── image_subscriber                   # 相机图像订阅
│   ├── pattern_classify                   # 数字分类
│   └── pnp_solver                        # IPPE/PnP 位姿估计
├── rcia_math_solver/                     # 位姿优化与束调整
│   ├── ba_solver                         # Ceres 束调整
│   └── math_solver_node                  # 解算器 ROS 2 节点
├── rcia_vision_tracker/                  # 目标跟踪与云台控制
│   ├── gimbal_control                     # 云台控制指令生成
│   ├── guard_dog_publish                 # 跟踪器心跳发布
│   ├── spinTop_predictor                 # 目标运动预测
│   ├── spinTop_tracker                   # 目标状态跟踪
│   ├── trajectory_compensator            # 弹道与延迟补偿
│   └── vision_tracker_node               # 跟踪器 ROS 2 节点
├── rcia_sensor_driver/                   # 硬件接入功能包
│   ├── rcia_camera_driver/               # 相机驱动功能包
│   │   └── camera_node                    # 相机 ROS 2 节点
│   └── rcia_serial_driver/               # 云台与串口驱动功能包
│       ├── protocol                       # 串口协议定义
│       ├── infantry_protocol              # 步兵协议实现
│       ├── serial_driver_node             # 串口 ROS 2 节点
│       └── uart_transporter               # UART 传输层
├── rcia_bringup/                         # 运行编排功能包
│   ├── config/node_params/                # 节点参数文件
│   └── launch/                            # 系统启动描述
├── rcia_vision_guard/                    # 守护与恢复节点
│   └── guard_dog                         # 守护实现
├── rcia_robot_description/               # 机器人模型功能包
│   ├── meshes/                            # 机器人网格资源
│   └── urdf/                              # URDF 与 Xacro 描述
├── rcia_vision_utils/                    # 轨迹工具与仿真
│   └── trajectory_simulation.py           # 轨迹仿真工具
├── ros2_foxglove_bridge/                 # Foxglove 可视化桥接
│   ├── generic_client                     # 通用 ROS 2 客户端
│   ├── message_definition_cache           # 消息定义缓存
│   ├── param_utils                        # 参数工具
│   ├── parameter_interface                # 参数接口
│   ├── ros2_foxglove_bridge               # Foxglove 桥接实现
│   ├── ros2_foxglove_bridge_node          # Foxglove 桥接节点
│   ├── smoke_test                          # 桥接冒烟测试
│   └── utils_test                          # 桥接工具测试
├── scripts/
│   ├── build/                            # 可复现构建入口
│   └── run/                              # 运行与可视化启动脚本
├── docs/                                 # 指令与运行维护文档
│   ├── notes/                            # 快捷指令参考
│   └── operations/                       # tmux 与运行规程
├── requirements.txt                      # 可选 Python 工具依赖
├── .gitignore                            # 构建、数据、模型和 IDE 排除项
├── LICENSE                               # MIT 许可证
├── README.md                             # 英文文档
└── README.zh.md                          # 中文文档
```

## 安装

### 环境要求

- Ubuntu 22.04 与 ROS 2 Humble。
- `colcon`、`rosdep`、C++17/C++20 工具链、OpenCV、Eigen 和 Ceres Solver。
- 用于分类的 ONNX Runtime，以及硬件运行所需的厂商相机 SDK。
- 用于可视化的 Foxglove Studio 与 `foxglove_bridge`。

### 初始化

```bash
git clone https://github.com/AcmeX/RCIA-Vision-2025.git
cd RCIA-Vision-2025
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r requirements.txt
```

如果 ONNX Runtime 不在 `/opt/onnxruntime`，请设置 `ONNXRUNTIME_ROOT`。如果使用可选 Intel IPP 构建路径，请设置 `ONEAPI_ROOT` 和 `IPP_ROOT_DIR`。相机 SDK 需要单独安装。

## 使用

构建核心功能包：

```bash
source /opt/ros/humble/setup.bash
bash scripts/build/build_workspace.sh
source install/setup.bash
```

启动感知与控制组合流水线：

```bash
ros2 launch rcia_bringup bringup.launch.py
```

按需在独立终端启动运行辅助组件：

```bash
bash scripts/run/run_guard.sh
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

功能包构建、快捷指令和 tmux 会话管理见 [docs/notes/quick_command.md](docs/notes/quick_command.md) 与 [docs/operations/tmux.md](docs/operations/tmux.md)。

## 许可证

本项目采用 MIT License，完整文本见 [LICENSE](LICENSE)。

## 联系方式

如有问题、issue或贡献：

- **GitHub Issues**: [https://github.com/AcmeX/RCIA-Vision-2025/issues](https://github.com/AcmeX/RCIA-Vision-2025/issues)
- **Email**: AcmeX@foxmail.com

---