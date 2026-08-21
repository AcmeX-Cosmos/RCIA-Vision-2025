// Standard
#include <iostream>
#include <vector>
#include <cmath>
#include <array>
#include <algorithm>
#include <functional>
#include <numeric>
#include <limits>


#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rcia_vision_interfaces/msg/trajectory.hpp"

using namespace std;
/**
 * @brief 弹道补偿类，用于计算射击角度补偿
 */
class Trajectory_compensator {
public:
    Trajectory_compensator(){};

    double calculate_compensation_angle(double range, double firing_angle_rad);

    void angle_compensator(const Eigen::Vector3d& target_position, double& pitch_rad, double& current_compen_height, double& current_distance);

    double bullet_mass;                                             // 子弹质量（单位：克）
    // double ballistic_coefficient;                                   // 弹道系数
    std::map<int, double> ballistic_coefficients; //= {
    //     {0, 0.45},
    //     {1, 0.355},
    //     {2, 0.245},
    //     {3, 0.175},
    //     {4, 0.125},
    //     {5, 0.085},
    //     {6, 0.055},
    //     {7, 0.038},
    //     {8, 0.0235},
    //     {9, 0.0185}
    // };

    double initial_velocity;
    std::vector<long int> keys;
    std::vector<double> values;
    

private:

    double acceleration_of_gravity = 9.8;
    
};


// 
// class BallisticSystem {
// public:

//     // 构造函数
//     BallisticSystem() :
//         node_(std::make_shared<rclcpp::Node>("vision_node")),
//         v0(26.0),                                // 初始速度 (m/s)
//         mass(0.003),                             // 子弹质量 (kg)
//         radius(0.0165),                          // 子弹半径 (m)
//         Cd(0.47),                                // 阻力系数（典型球形弹丸）
//         rho(1.225),                              // 空气密度 (kg/m³)
//         g(9.80665),                              // 重力加速度 (m/s²)
//         camera_offset({0.0, 0.0, 0.0}),          // 坐标系参数（单位：米）
//         R(Eigen::Matrix3d::Identity()),          // 假设摄像头与枪管无旋转

//         // 弹道计算参数
//         dt(0.01),                                // 积分步长 (s)
//         max_flight_time(0.5)                     // 最大飞行时间 (s)
//     {
//         A = M_PI * radius * radius;              // 预计算弹丸截面积

//         trajectory_publisher_ = node_->create_publisher<rcia_vision_interfaces::msg::Trajectory>("trajectory_topic", 10);

//         trajectoryMarker_publish_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 10);

//         // 初始化实测数据      
//         // [0] 角度补偿高度, [1] 枪口到装甲板距离
//         realData = {
//             {0.157227, 0.5},
//             {0.314515, 1.0}, 
//             {0.47186, 1.5},
//             {0.629262, 2.0},
//             {0.786717, 2.5},
//             {1.10178, 3.0},
//             {1.25938, 3.5},
//             {1.25938, 4.0},
//             {1.41702, 4.5},
//             {1.57471, 5.0},
//             {1.73243, 5.5},
//             {1.89019, 6.0},
//             {2.04799, 6.5},
//             {2.20582, 7.0},
//         };

//         // 校准阻力系数
//         calibrateCd(realData);

//     }

//     std::vector<std::pair<double,  double>> realData;

//     // 将摄像头坐标转换为枪口坐标系
//     Eigen::Vector3d transformCoordinates(const Eigen::Vector3d& cameraPoint) const;

//     template <typename T>
//         T dynamics(T v) const;
        
//     // 四阶龙格-库塔法（RK4）
//     template <typename T>
//         void rk4_solver(T& x, T& y, T& vx, T& vy, T ax, T ay, T dt);

//     std::pair<double, std::vector<std::array<double, 4>>> simulate(double theta_deg, const Eigen::Vector3d& targetCam, bool verbose = true, const double& delta_pitch = 0.0);
    
//     // 定义误差函数
//     double errorFunc(const Eigen::VectorXd& Cd, const std::vector<std::pair<double, double>>& realData);

//     // 根据实测数据校准阻力系数
//     void calibrateCd(const std::vector<std::pair<double, double>>& realData);

//     // 自动解算最佳仰角
//     std::pair<double, double> autoAim(const Eigen::Vector3d& targetCam, const std::string& method = "hybrid", const double& delta_pitch = 0.0, const double& compensation_height = 0.0);
    
//     void visualize_trajectory(const std::vector<std::array<double, 4>>& trajectory, cv::Mat& img, const Mat& rvec, const Mat& tvec, const Eigen::Vector3d& targetCam, const double& optimal_angle, double delta_pitch);

//     void publishTrajectoryToPython(const std::vector<std::array<double, 4>>& trajectory, const Eigen::Vector3d& targetCam);
    
//     void addTrajectoryMarkerMsg(const Mat& publishRvec, const Mat& publishTvec, const int& publishIndex);

//     void publishTrajectoryMarkerArray();

// public:
//     double Cd;                      // 阻力系数（典型球形弹丸）
    
//     double v0;                      // 初始速度 (m/s)
//     double mass;                    // 子弹质量 (kg)
//     double radius;                  // 子弹半径 (m)
//     double rho;                     // 空气密度 (kg/m³)
//     double g;                       // 重力加速度 (m/s²)
    
//     Eigen::Vector3d camera_offset;  // [dx, dy, dz]
//     Eigen::Matrix3d R;              // 假设摄像头与枪管无旋转

//     double dt; // 积分步长
//     double max_flight_time;         // 最大飞行时间

//     double A;                       // 弹丸截面积

// private:

//     // 小弹丸直径
//     static constexpr float SMALL_BULLET_DDIAMETER = 16.5;

//     double small_radius = SMALL_BULLET_DDIAMETER / 2.0 / 10.0;

//     // 定义子弹轴点
//     std::vector<cv::Point3f> axisBullet = {
//         cv::Point3f(-small_radius, -small_radius, -small_radius),  // 点 1
//         cv::Point3f(-small_radius, small_radius, -small_radius),   // 点 2
//         cv::Point3f(small_radius, small_radius, -small_radius),    // 点 3
//         cv::Point3f(small_radius, -small_radius, -small_radius),   // 点 4

//         cv::Point3f(-small_radius, -small_radius, small_radius),  // 点 5
//         cv::Point3f(-small_radius, small_radius, small_radius),   // 点 6
//         cv::Point3f(small_radius, small_radius, small_radius),    // 点 7
//         cv::Point3f(small_radius, -small_radius, small_radius)    // 点 8
//     };

//     rclcpp::Node::SharedPtr node_;

//     rclcpp::Publisher<rcia_vision_interfaces::msg::Trajectory>::SharedPtr trajectory_publisher_;
    
//     visualization_msgs::msg::MarkerArray trajectoryMarkerArray;
	
// 	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectoryMarker_publish_;
// };


// // 计算 yaw 轴所差 弧度
// template <typename T>
// inline T calculate_deltaYaw(const cv::Mat& armorTvecs) {
//     return atan2(armorTvecs.at<double>(0), armorTvecs.at<double>(2));
// }

// // 计算 pitch 轴所差 弧度
// template <typename T>
// inline T calculate_deltaPitch(const cv::Mat& armorTvecs) {
//     return atan2(armorTvecs.at<double>(1), armorTvecs.at<double>(2));
// }