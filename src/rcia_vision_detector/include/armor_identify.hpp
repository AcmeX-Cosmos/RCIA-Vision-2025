#ifndef ARMOR_IDENTIFY_HPP
#define ARMOR_IDENTIFY_HPP

// The following include can be optimized
#include "Types.hpp"
#include "armor_detector.hpp"
#include "pnp_solver.hpp"
#include "pattern_classify.hpp"

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_parameters.hpp>
#include <rclcpp/parameter.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Debug
#include "debug.hpp"

namespace rcia::vision_identify {
class ArmorIdentifier : public rclcpp::Node {
public:

    explicit ArmorIdentifier(const rclcpp::NodeOptions &options);

    std::unique_ptr<ArmorDetector> init_ArmorDetector();

    std::unique_ptr<ArmorDetector> armor_detector_;

    rcia_vision_interfaces::msg::ArmorIdentifyInfo armor_identify_msg;
    rcia_vision_interfaces::msg::GimbalCurrentInfo gimbal_current_msg;

    void identify_armor(Mat& img, const rcia::SerialProtocol::SerialDataStruct *serial_data_ptr);

protected:
   
    void decide_range(Mat& src_img);

    void select_optimal_armor(
        vector<pair<vector<Point>, string>>& armor_candidates_);


    void display_debug_info(
        const rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr,
        const string& gimbal,
        const double& fps);

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

private:

    void _initialize_identify(Mat& img, const rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr);

    void _initialize_patrol(const double& image_height, const double& image_width) ;

    void _initialize_control(
        const rcia::SerialProtocol::SerialDataStruct *serial_data_ptr,
        struct rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr);

    void tracker_state_callback(const std::shared_ptr<rcia_vision_interfaces::msg::TrackerState> tracker_state_msg);

    void target_info_callback(const std::shared_ptr<rcia_vision_interfaces::msg::TargetSpinTop> target_info_msg);

    void target_pose_transform(const std::shared_ptr<rcia_vision_interfaces::msg::TargetSpinTop> target_info_msg, 
                               TargetSpinTopStruct& target_pose_info_);

private:
    // The following variables can be optimized
    bool is_initialized;

    int image_width, image_height, last_armor_width, last_armor_height;
    
    uint8_t armorIdx;
    double armorAcc;
    
    int isDebugModeEnabled_;
    int detection_count_;

    int detector_start_x, detector_start_y, detector_end_x, detector_end_y;
    int detector_center_x, detector_center_y;

    int last_center_x, last_center_y;

    int last_compensation_height;
   
    string tracker_state_;
    double flight_time;

    int last_set_yaw_angle, last_set_pitch_angle;

    Mat lights, draw_img;

    Mat target_central_tvec;  // temporary defined

    vector<int> aim_target_points;

    vector<string> debug_info_text;
    vector<pair<vector<Point>, string>> armor_candidates_;

    PatternClassifier ptClassifier;

    // Struct
    GimbalInfoStruct gimbal;
    TargetSpinTopStruct target_pose_info_;  // 用于调整决策范围 还有调试

    // ROS2 Nodes
    rclcpp::Node::SharedPtr node_;

    // ROS2 shared_ptr
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // ROS2 Publisher
    rclcpp::Publisher<rcia_vision_interfaces::msg::ArmorIdentifyInfo>::SharedPtr armor_identify_info_pub_;
    rclcpp::Publisher<rcia_vision_interfaces::msg::GimbalCurrentInfo>::SharedPtr gimbal_current_info_pub_;

    // ROS2 Subscriber
    rclcpp::Subscription<rcia_vision_interfaces::msg::TrackerState>::SharedPtr tracker_state_sub_;
    rclcpp::Subscription<rcia_vision_interfaces::msg::TargetSpinTop>::SharedPtr target_info_sub_;
    rclcpp::Subscription<rcia_vision_interfaces::msg::SerialReceiveData>::SharedPtr serial_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    // ROS2 msgidentify_armor
    rcia_vision_interfaces::msg::TargetSpinTop latest_target_info_;

    // ROS2 paramters
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_paramters_callback_handle_;
    rcl_interfaces::msg::SetParametersResult on_set_parameters(const std::vector<rclcpp::Parameter> &parameters);

    // time
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;
    int frame_count_;
    double fps_;

    // 原子指针指向当前读取的缓冲区
    std::atomic<rcia::SerialProtocol::SerialDataStruct *> current_read_buffer_;
    rcia::SerialProtocol::SerialDataStruct buffer1_, buffer2_; // 双缓冲实例

};  
}

#endif