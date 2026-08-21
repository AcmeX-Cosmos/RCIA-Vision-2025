# Quick Command Reference

本文件只保留构建、运行和诊断所需的常用命令。每个命令组先说明用途，再给出可直接执行的指令。

## 1. 环境初始化

用途：加载 ROS 2 和当前工作区环境；每个新终端执行一次。

```bash
cd /path/to/RCIA-Vision-2025
source /opt/ros/humble/setup.bash
source install/setup.bash
```

用途：首次配置或补齐 ROS 依赖。

```bash
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r requirements.txt
```

## 2. 清理与构建

用途：删除本工作区生成的构建结果，解决包索引或 ABI 状态不一致问题。

```bash
rm -rf build install log
```

用途：构建接口、工具、守护和检测核心包。

```bash
source /opt/ros/humble/setup.bash
bash scripts/build/build_workspace.sh
```

用途：只构建检测器；可通过 `ONNXRUNTIME_ROOT`、`ONEAPI_ROOT` 和 `IPP_ROOT_DIR` 指定本机依赖路径。

```bash
source /opt/ros/humble/setup.bash
ONNXRUNTIME_ROOT=/opt/onnxruntime bash scripts/build/build_detector.sh
```

用途：按需构建指定 ROS 2 包，并让 `colcon` 自动处理包依赖。

```bash
colcon build --symlink-install --packages-up-to rcia_bringup
```

## 3. 节点与系统启动

用途：启动完整的感知、解算、跟踪和控制流水线；执行前必须完成构建并加载工作区。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rcia_bringup bringup.launch.py
```

用途：单独启动守护节点，用于监测心跳和恢复运行窗口。

```bash
ros2 run rcia_vision_guard guard_dog
```

用途：检查当前可发现的包、节点、话题和服务。

```bash
ros2 pkg list | rg 'rcia_vision|foxglove'
ros2 node list
ros2 topic list
ros2 service list
```

## 4. Foxglove 与数据诊断

用途：启动 Foxglove Bridge，为 Foxglove Studio 提供 ROS 2 数据流。

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

用途：检查指定话题的消息类型、发布者和实时数据。

```bash
ros2 topic type /armor_identify_info
ros2 topic info /armor_identify_info --verbose
ros2 topic echo /armor_identify_info --once
```

## 5. Git 工作流

用途：查看变更并提交经过验证的修改；默认分支为 `main`。

```bash
git status
git diff --check
git add .
git commit -m "Describe the change"
git push origin main
```

## 6. 运行入口

用途：使用仓库提供的脚本启动桌面会话、系统启动、守护和 Foxglove；脚本会自动解析仓库根目录。

```bash
bash scripts/run/run_bringup.sh
bash scripts/run/run_guard.sh
bash scripts/run/run_foxglove.sh
bash scripts/run/run_desktop.sh
```

tmux 会话、窗口和面板操作见 [docs/operations/tmux.md](../operations/tmux.md)。
