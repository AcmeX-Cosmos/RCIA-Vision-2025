#!/bin/bash

if [ "$DISPLAY" = ":0" ]; then

    SCRIPT_DIR=$(dirname "$(realpath "$0")")
    WORKSPACE_ROOT=$(realpath "$SCRIPT_DIR/../..")
    cd "$WORKSPACE_ROOT" || exit 1

    # 创建tmux会话（如果不存在）
    if ! tmux has-session -t rcia_vision 2>/dev/null; then
    	tmux new-session -s rcia_vision -d
    	sleep 1
    fi

    # 启动视觉节点（确保窗口保持打开）
    source /opt/ros/humble/setup.bash && 
    source ${WORKSPACE_ROOT}/install/setup.bash &&
    ros2 launch rcia_bringup bringup.launch.py

else
    echo "Not running in an $DISPLAY 0, skipping ROS 2 publisher startup..."
fi

