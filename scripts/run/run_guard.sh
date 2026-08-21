#!/bin/bash

# 检查当前的$DISPLAY是否为0（通常表示本地的图形界面登录）
if [ "$DISPLAY" = ":0" ]; then
    echo "Running on $DISPLAY 0 session, starting ROS 2 publisher..."
    SCRIPT_DIR=$(dirname "$(realpath "$0")")
    WORKSPACE_ROOT=$(realpath "$SCRIPT_DIR/../..")
    cd "$WORKSPACE_ROOT" || exit 1

    # 更简洁的tmux会话管理
    if ! tmux has-session -t rcia_vision 2>/dev/null; then
    tmux new-session -s rcia_vision -d
    fi

    sleep 0.5

    # 启动视觉节点
    source /opt/ros/humble/setup.bash &&
    source ${WORKSPACE_ROOT}/install/setup.bash &&
    ros2 run rcia_vision_guard guard_dog
    
else
    echo "Not running in an $DISPLAY 0, skipping ROS 2 publisher startup..."
fi

