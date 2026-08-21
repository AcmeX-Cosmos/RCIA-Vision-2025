// vision_tracker_node.hpp

#ifndef VISION_TRACKER_NODE_HPP
#define VISION_TRACKER_NODE_HPP

#include "Type.hpp"

// ROS2
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>

#include "spinTop_tracker.hpp"
#include "gimbal_control.hpp"
#include "guard_dog_publish.hpp"

namespace rcia{
namespace vision_tracker{
using tf2_filter = tf2_ros::MessageFilter<rcia_vision_interfaces::msg::ArmorBaposeInfo>;

class VisionTrackerNode : public rclcpp::Node {
public:
    explicit VisionTrackerNode(const rclcpp::NodeOptions & options);

protected:

    void serial_receive_callback(std::shared_ptr<rcia_vision_interfaces::msg::SerialReceiveData> msg);

    void solver_callback(const rcia_vision_interfaces::msg::ArmorBaposeInfo::SharedPtr &armor_bapose_msg_);

    void armor_pose_transform(const rcia_vision_interfaces::msg::ArmorBaposeInfo::SharedPtr &armor_bapose_msg_, rcia_vision_interfaces::msg::OdomMeasurement &odom_measurement_msg_);

    void publishMarkers(const rcia_vision_interfaces::msg::TargetSpinTop &spinTop_target_msg);

    void publishTrackerState();

private:
// normal variable

    bool debug_mode_;

    double lost_time_thresh_;

    std::string target_frame_; 

    std::unique_ptr<SpinTopTracker> tracker_;

    std::unique_ptr<GimbalControl> control_;

    std::unique_ptr<HeartBeatPublisher> heartbeat_;

private:
// ROS private variable

    // ROS2 shared_ptr
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    std::shared_ptr<tf2_filter> tf2_filter_;

    // ROS2 msg
    rcia_vision_interfaces::msg::ArmorBaposeInfo armor_bapose_msg;
    rcia_vision_interfaces::msg::SerialReceiveData latest_serial_data_;
    rcia_vision_interfaces::msg::SerialTransmitData vision_control_msg_;
    
    // ROS2 pub
    rclcpp::Publisher<rcia_vision_interfaces::msg::OdomMeasurement>::SharedPtr odom_measurement_pub_;
    rclcpp::Publisher<rcia_vision_interfaces::msg::TargetSpinTop>::SharedPtr spin_top_info_pub_;
    rclcpp::Publisher<rcia_vision_interfaces::msg::SerialTransmitData>::SharedPtr vision_data_pub_;
    rclcpp::Publisher<rcia_vision_interfaces::msg::TrackerState>::SharedPtr tracker_state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // ROS2 sub
    message_filters::Subscriber<rcia_vision_interfaces::msg::ArmorBaposeInfo> armor_bapose_sub_;

    // ROS Time
    rclcpp::Time watchDog_previous_time_;

    // 临时定义
    rclcpp::Subscription<rcia_vision_interfaces::msg::SerialReceiveData>::SharedPtr serial_receive_sub_;
    std::mutex serial_data_mutex_;
    
    // ROS2 marker 可视化标记发布
    visualization_msgs::msg::Marker position_marker_;
    visualization_msgs::msg::Marker armors_marker_;
};


}
}


#endif // VISION_TRACKER_NODE_HPP