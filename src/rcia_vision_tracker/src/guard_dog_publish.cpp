#include "guard_dog_publish.hpp"


HeartBeatPublisher::HeartBeatPublisher(std::weak_ptr<rclcpp::Node> n) : node_(n) {
    auto node_shared = node_.lock();
    heartbeat_pub_ = node_shared->create_publisher<rcia_vision_interfaces::msg::HeartBeat>("heartbeat", 10);

    node_shared.reset();
}

void HeartBeatPublisher::publish_heartbeat() {
    auto node_shared = node_.lock();
        if (node_shared) {
        static auto last_publish_time = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();

        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_publish_time).count() >= 1.0) {
            auto heartbeat_msg = rcia_vision_interfaces::msg::HeartBeat();
            heartbeat_msg.heartbeat_id = 1;
            
            heartbeat_msg.stamp = node_shared->now();
            
            heartbeat_pub_->publish(heartbeat_msg);
            
            last_publish_time = current_time;
        }
    }
}


void HeartBeatPublisher::refresh(rcia_vision_interfaces::msg::SerialTransmitData vision_control_msg_) {
    fps_count_ ++;
    if (fps_count_ > 200) {
        cout << "\n\nfps: " << fps << endl;
        cout << "pitch1: " << vision_control_msg_.pitch_angle << " | yaw1: " << vision_control_msg_.yaw_angle 
            << " | find_flag: " << vision_control_msg_.find_flag << " | fire_flag: " << vision_control_msg_.fire_flag
            << " | time_stamp: " << vision_control_msg_.time_stamp << " | distance: " << vision_control_msg_.distance << endl;

        fps_count_ = 0;

        auto now = chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto milliseconds = duration_cast<chrono::milliseconds>(duration).count();

        now_time_ = static_cast<double>(milliseconds) / 1000.0;

        fps = 200 / (now_time_ - last_time_);
        
        last_time_ = now_time_;

        publish_heartbeat();
        
    }
}
