#!/bin/sh

if [ "$DISPLAY" = ":0" ]; then
    #vncserver -kill :2
    sleep 0.25

    # 启动 VNC 服务器
    #vncserver :1 -geometry 1280x768 -depth 16 -localhost no
    tigervncserver :2 -geometry 1920x1080 -depth 24 -localhost no
    
    tmux attach -t rcia_vision
fi







#gnome-terminal --title="VNC" -- bash -c "bash scripts/run/run_desktop.sh; exec bash"
#gnome-terminal --title="rosbridge"   -- bash -c "bash scripts/run/run_foxglove.sh; exec bash"
#gnome-terminal --title="Vision" -- bash -c "bash scripts/run/run_bringup.sh; exec bash"
#gnome-terminal --title="WatchDog0"   -- bash -c "bash scripts/run/run_guard.sh; exec bash"
