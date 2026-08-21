# tmux Operations

职责：为 ROS 2 启动、守护和 Foxglove 进程提供可恢复的终端会话。以下命令默认使用会话名 `rcia_vision`。

## 1. 会话管理

用途：创建后台会话，适合启动长期运行的 ROS 2 进程。

```bash
tmux new-session -d -s rcia_vision
```

用途：列出当前用户的 tmux 会话并重新连接已有会话。

```bash
tmux list-sessions
tmux attach-session -t rcia_vision
```

用途：在退出终端前脱离会话，或结束指定会话。

```text
Ctrl-b d
tmux kill-session -t rcia_vision
```

## 2. 窗口管理

用途：为不同 ROS 2 进程创建独立窗口、切换窗口和查看窗口列表。

```bash
tmux new-window -t rcia_vision -n Vision
tmux new-window -t rcia_vision -n Guard
tmux list-windows -t rcia_vision
tmux select-window -t rcia_vision:Vision
```

用途：关闭指定窗口；窗口名或窗口编号均可使用。

```bash
tmux kill-window -t rcia_vision:Guard
```

## 3. 面板管理

用途：在同一个窗口中分屏运行相关命令，并在面板间切换。

```text
Ctrl-b %       # 垂直分割
Ctrl-b "       # 水平分割
Ctrl-b o       # 切换面板
Ctrl-b x       # 关闭当前面板
```

用途：从命令行创建带有预设布局的面板。

```bash
tmux split-window -h -t rcia_vision:Vision
tmux split-window -v -t rcia_vision:Vision
tmux list-panes -t rcia_vision:Vision
```

## 4. 常用会话布局

用途：创建标准窗口并分别启动系统、守护和 Foxglove 入口。命令在仓库根目录执行。

```bash
tmux new-session -d -s rcia_vision -n Vision
tmux new-window -t rcia_vision -n Guard
tmux new-window -t rcia_vision -n Foxglove
tmux send-keys -t rcia_vision:Vision 'bash scripts/run/run_bringup.sh' C-m
tmux send-keys -t rcia_vision:Guard 'bash scripts/run/run_guard.sh' C-m
tmux send-keys -t rcia_vision:Foxglove 'bash scripts/run/run_foxglove.sh' C-m
tmux attach-session -t rcia_vision
```

用途：确认会话和窗口是否存在；适合脚本故障排查。

```bash
tmux has-session -t rcia_vision
tmux list-windows -t rcia_vision
tmux capture-pane -p -t rcia_vision:Vision
```
