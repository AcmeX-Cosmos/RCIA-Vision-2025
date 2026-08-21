#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "rcia_vision_interfaces/msg/heart_beat.hpp"

using rcia_vision_interfaces::msg::HeartBeat;
using namespace std;
using namespace std::chrono_literals;

class GuardDog : public rclcpp::Node {
public:
    GuardDog() : Node("guard_dog") {
        // 确保tmux服务器运行
        if (execute_command("tmux start-server") != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start tmux server");
        }
        this_thread::sleep_for(1s);

        // 检查并创建会话（不杀死现有会话）
        if (execute_command("tmux has-session -t rcia_vision") != 0) {
            if (execute_command("tmux new-session -d -s rcia_vision") != 0) {
                RCLCPP_FATAL(this->get_logger(), "Failed to create tmux session");
                rclcpp::shutdown();
                return;
            }
            this_thread::sleep_for(1s);
        }

        subscription_ = this->create_subscription<rcia_vision_interfaces::msg::HeartBeat>(
            "heartbeat", 10,
            [this](const rcia_vision_interfaces::msg::HeartBeat::SharedPtr msg) {
                this->heartbeat_callback(msg);
            });

        timer_ = this->create_wall_timer(
            250ms, std::bind(&GuardDog::check_heartbeat, this));

        RCLCPP_INFO(this->get_logger(), "Guard dog is ready...");
    }

private:
    int execute_command(const string& cmd) {
        string full_cmd = cmd + " 2>/dev/null";
        int result = system(full_cmd.c_str());
        if (result != 0) {
            RCLCPP_DEBUG(this->get_logger(), "Command failed: %s (code: %d)", 
                        cmd.c_str(), result);
        }
        return result;
    }

    void heartbeat_callback(const rcia_vision_interfaces::msg::HeartBeat::SharedPtr msg) {
        last_heartbeat_time_ = this->now();
        RCLCPP_DEBUG(this->get_logger(), "Received heartbeat: Timestamp=%.3f", 
                     msg->stamp.sec + msg->stamp.nanosec * 1e-9);
    }

    void check_heartbeat() {
        auto now = this->now();
        auto duration = now - last_heartbeat_time_;

        if (duration.seconds() > 5.0 && !pause_) {
            pause_ = true;
            RCLCPP_ERROR(this->get_logger(), 
                        "HeartBeat timeout! No heartbeat for %.1f seconds.", 
                        duration.seconds());

            // 重启Vision节点
            restart_vision_node();
            
            // 重启自身
            restart_self();

            pause_ = false;
        }
    }

    void restart_vision_node() {
        // 查找并杀死现有Vision窗口
        
            execute_command("tmux kill-window -t rcia_vision:Vision");
            this_thread::sleep_for(250ms);
        
        // 创建新Vision窗口
        execute_command("tmux new-window -t rcia_vision -n Vision -d \
                       'bash scripts/run/run_bringup.sh'");
        this_thread::sleep_for(250ms);
    }

   void restart_self() {
    // 检测当前存在的WatchDog窗口
    bool watchdog0_exists = (execute_command("tmux list-windows -t rcia_vision -F '#{window_name}' | grep -x 'WatchDog0'") == 0);
    bool watchdog1_exists = (execute_command("tmux list-windows -t rcia_vision -F '#{window_name}' | grep -x 'WatchDog1'") == 0);

    // 确定要创建的窗口名称
    std::string new_window_name;
    if (watchdog0_exists) {
        // 如果WatchDog0存在，则创建WatchDog1
        new_window_name = "WatchDog1";
        execute_command("tmux new-window -t rcia_vision -n WatchDog1 -d 'bash scripts/run/run_guard.sh'");
        // 先关闭WatchDog0
        execute_command("tmux kill-window -t rcia_vision:WatchDog0");
    } else if (watchdog1_exists) {
        execute_command("tmux new-window -t rcia_vision -n WatchDog0 -d 'bash scripts/run/run_guard.sh'");
        // 如果WatchDog1存在，则创建WatchDog0
        new_window_name = "WatchDog0";
        // 先关闭WatchDog1
        execute_command("tmux kill-window -t rcia_vision:WatchDog1");
    } else {
        // 如果都不存在，默认创建WatchDog0
        execute_command("tmux new-window -t rcia_vision -n WatchDog1 -d 'bash scripts/run/run_guard.sh'");
    }

    this_thread::sleep_for(250ms);

    // 创建新窗口

    RCLCPP_INFO(this->get_logger(), "Restarted %s window", new_window_name.c_str());

    // 关闭当前节点
        rclcpp::shutdown();
    }

    rclcpp::Subscription<rcia_vision_interfaces::msg::HeartBeat>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_heartbeat_time_ = this->now();
    bool pause_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GuardDog>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
