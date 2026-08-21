#ifndef DEBUG_HPP
#define DEBUG_HPP

// Standard
#include <chrono>

#include <vision_initialize.hpp>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rcia_vision_interfaces/msg/heart_beat.hpp" 

#include "Types.hpp"

using namespace chrono;


double get_current_timestamp();


void getDebugInfoText(
    vector<string>& debug_info_text,
    const struct rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr,
    const string& aim_mode,
    const int& detection_count_,
    const double& flight_time,
    const string& gimbal,
    const double& fps, 
    rcia_vision_interfaces::msg::GimbalCurrentInfo& gimbal_current_msg, 
    rcia_vision_interfaces::msg::ArmorIdentifyInfo& armor_identify_msg);


void debugPutText(
    const Mat& draw_img,
    const vector<string>& debug_info_text,
    const int& image_height,
    const int& image_width);


void debug_draw_image(
    Mat& draw_img,
    const vector<int>& aim_target_points,
    const string& gimbal,
    const int& image_height,
    const int& image_width);


void debug_target_pose(Mat& draw_img, const rcia::vision_identify::TargetSpinTopStruct& target_pose_info_, rcia_vision_interfaces::msg::ArmorIdentifyInfo& armor_identify_msg);

void reproject_and_draw(Mat &draw_img, std::vector<cv::Point2f> imgpts, std::string draw_type);

#endif