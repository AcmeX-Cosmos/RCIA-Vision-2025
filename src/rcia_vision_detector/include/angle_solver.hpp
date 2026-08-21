#ifndef ANGLE_SOLVER_HPP
#define ANGLE_SOLVER_HPP

#include "pnp_solver.hpp"
#include "Types.hpp"

// 移动平均滤波器类，用于平滑数据
class MovingAverageFilter {
public:
    explicit MovingAverageFilter(size_t windowSize) : windowSize_(windowSize) {}

    // 更新滤波器并返回当前平均值
    double update(double newValue);
    
private:
    size_t windowSize_; // 窗口大小
    std::deque<double> values_; // 存储值的队列
    double total_ = 0; // 当前窗口内值的总和
};


// 将像素坐标转换为角度
void pixel_to_angle(
    const struct rcia::vision_identify::GimbalInfoStruct::CameraInfoStruct& camera_info,
    struct rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr,
    Mat& armor_rvec,
    Mat& armor_tvec,
    String transformType,
    rcia_vision_interfaces::msg::GimbalCurrentInfo& gimbal_current_msg,
    rcia_vision_interfaces::msg::ArmorIdentifyInfo& armor_identify_msg);


// 根据上次设置的角度和补偿高度，决定像素坐标
void decide_angle_to_pixel(
    const struct rcia::vision_identify::GimbalInfoStruct::CameraInfoStruct& camera_info,
    const Mat& target_central_tvec,
    int& last_center_x,
    int& last_center_y);


// 预测角度转换为像素坐标
void predict_angle_to_pixel(
    const struct rcia::vision_identify::GimbalInfoStruct::CameraInfoStruct& camera_info,
    struct rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr,
    rcia_vision_interfaces::msg::GimbalCurrentInfo& gimbal_current_msg);

#endif