#include "vision_initialize.hpp"

#include "angle_solver.hpp"
#include "Types.hpp"


// 移动平均滤波器类，用于平滑数据
double MovingAverageFilter::update(double newValue) {
    if (values_.size() == windowSize_) {
        total_ -= values_.front();
        values_.pop_front();
    }

    values_.push_back(newValue);
    total_ += newValue;

    return total_ / values_.size();
}

MovingAverageFilter filter1(50);

/**
 * @brief 将像素坐标转换为角度
 * 
 * 该函数根据给定的最佳装甲板轮廓点集合、装甲板类型、相机信息、识别信息结构体指针、装甲板旋转向量、装甲板平移向量、相机位置和变换类型，
 * 计算云台的角度信息。
 * 
 * @param best_armor_contours_ 最佳装甲板轮廓点集合
 * @param best_armor_type 最佳装甲板类型
 * @param camera_info 相机信息结构体
 * @param identify_info_ptr 识别信息结构体指针
 * @param armor_rvec 装甲板旋转向量
 * @param armor_tvec 装甲板平移向量
 * @param transformType 变换类型
 */
void pixel_to_angle(
    const struct rcia::vision_identify::GimbalInfoStruct::CameraInfoStruct& camera_info,
    struct rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr,
    Mat& armor_rvec,
    Mat& armor_tvec,
    String transformType,
    rcia_vision_interfaces::msg::GimbalCurrentInfo& gimbal_current_msg,
    rcia_vision_interfaces::msg::ArmorIdentifyInfo& armor_identify_msg)
{

    solve_pnp(camera_info, armor_rvec, armor_tvec, armor_identify_msg);

    angle_solver(gimbal_current_msg, armor_identify_msg);

    gimbal_current_msg.diff_yaw_sx100 = gimbal_current_msg.diff_yaw * 100.0;
    gimbal_current_msg.diff_pitch_sx100 = gimbal_current_msg.diff_pitch * 100.0;
    
}


/**
 * @brief 根据上次设置的角度和补偿高度，决定像素坐标
 * 
 * 该函数根据上次设置的俯仰角、偏航角和补偿高度，计算出对应的像素坐标。
 * 
 * @param camera_info 相机信息结构体
 * @param identify_info_ptr 识别信息结构体指针
 * @param last_Pitch_set_angle 上次设置的俯仰角
 * @param last_Yaw_set_angle 上次设置的偏航角
 * @param last_compensation_height 上次补偿的高度
 * @param last_center_x 上次计算的中心点x坐标（输出参数）
 * @param last_center_y 上次计算的中心点y坐标（输出参数）
 */
// void decide_angle_to_pixel(
//     const struct rcia::vision_identify::GimbalInfoStruct::CameraInfoStruct& camera_info,
//     const Mat& target_central_tvec,
//     int& last_center_x,
//     int& last_center_y)
// {   


//     // 更新输出参数
//     last_center_x = static_cast<int>(image_points[0].x);
//     last_center_y = static_cast<int>(image_points[0].y);

// }



/**
 * @brief 预测角度转换为像素坐标
 * 
 * 该函数根据识别信息中的俯仰角和偏航角，预测对应的像素坐标。
 * 
 * @param camera_info 相机信息结构体
 * @param identify_info_ptr 识别信息结构体指针
 */
void predict_angle_to_pixel(
    const struct rcia::vision_identify::GimbalInfoStruct::CameraInfoStruct& camera_info,
    struct rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr,
    rcia_vision_interfaces::msg::GimbalCurrentInfo& gimbal_current_msg)
{
    // 预测角度值
    double prediction_pitch = -(gimbal_current_msg.set_pitch_angle) / 100.0;

    double prediction_yaw = -(gimbal_current_msg.set_yaw_angle) / 100.0;

    point_solver({prediction_yaw, prediction_pitch}, camera_info, identify_info_ptr);
}