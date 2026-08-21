#ifndef TYPES_HPP
#define TYPES_HPP

#include <iostream>
#include <string>
#include <algorithm>
#include <array>

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>
#include <angles/angles.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <cmath>

#include <gflags/gflags.h>
#include <glog/logging.h>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// ROS2 msg
#include "rcia_vision_interfaces/msg/armor_identify_info.hpp"
#include "rcia_vision_interfaces/msg/armor_bapose_info.hpp"
#include "rcia_vision_interfaces/msg/target_spin_top.hpp"

//using namespace
using namespace std;
using namespace cv;
using namespace Eigen;



namespace PNP {
    // 小装甲板的宽度（单位：毫米）
    static constexpr float SMALL_ARMOR_WIDTH = 133.0;
    // 小装甲板的高度（单位：毫米）
    static constexpr float SMALL_ARMOR_HEIGHT = 50.0;
    // 大装甲板的宽度（单位：毫米）
    static constexpr float LARGE_ARMOR_WIDTH = 214.0;
    // 大装甲板的高度（单位：毫米）
    static constexpr float LARGE_ARMOR_HEIGHT = 52.0;

    // 计算小装甲板的半宽和半高（单位：米）
    constexpr double small_half_x = SMALL_ARMOR_WIDTH / 2.0 / 1000.0; // 半宽
    constexpr double small_half_y = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0; // 半高
    // 计算大装甲板的半宽和半高（单位：米）
    constexpr double large_half_x = LARGE_ARMOR_WIDTH / 2.0 / 1000.0; // 半宽
    constexpr double large_half_y = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0; // 半高
    
    // 定义小装甲板的3D点集（世界坐标系中的四个角点）
    static const vector<Point3f> object_3d_points_small{
        Point3f(-small_half_x, -small_half_y, 0), // 左上角 (tl)
        Point3f(small_half_x, -small_half_y, 0),  // 右上角 (tr)
        Point3f(small_half_x, small_half_y, 0),   // 右下角 (br)
        Point3f(-small_half_x, small_half_y, 0)   // 左下角 (bl)
    };

    // 定义大装甲板的3D点集（世界坐标系中的四个角点）
    static const vector<Point3f> object_3d_points_large{
        Point3f(-large_half_x, -large_half_y, 0), // 左上角 (tl)
        Point3f(large_half_x, -large_half_y, 0),  // 右上角 (tr)
        Point3f(large_half_x, large_half_y, 0),   // 右下角 (br)
        Point3f(-large_half_x, large_half_y, 0)   // 左下角 (bl)
    };

    // 黑云台相机的内参矩阵
    static const Mat black_camera_matrix = (Mat_<double>(3, 3) <<
       1557.112832049196, 0, 640.0984500155263, // fx, cx
        0, 1735.09586187522, 512.0025696784391, // fy, cy
        0.00000000e+00, 0.00000000e+00, 1.00000000e+00 // 填充项
    );

    // 黑云台相机的畸变系数
    static const Mat black_dist_coeffs = (Mat_<double>(1, 5) <<
        0.02379281673136642,  -2.931918199663403,   -0.0001751467257614772,  0.001130183050415235,  42.30457985450274
    );

    // 白云台相机的内参矩阵
    static const Mat white_camera_matrix = (Mat_<double>(3, 3) <<
        2033.34590715899, 0, 4.80508726e+02, // fx, cx
        0, 2031.850277899148, 3.60779722e+02, // fy, cy
        0.00000000e+00, 0.00000000e+00, 1.00000000e+00 // 填充项
    );

    // 白云台相机的畸变系数
    static const Mat white_dist_coeffs = (Mat_<double>(1, 5) <<
        -0.07021873457194938, -0.7116172452104839, -0.001027277978918068, -5.16205243972664e-05, 12.86533239702043
    );
}

namespace Reprojection {
    static const Mat armor_axis = (cv::Mat_<float>(8, 3) << 
        -6.9, -2.9, 0, -6.9, 2.9, 0, 6.9, 2.9, 0, 6.9, -2.9, 0,
        -6.9, -2.9, 1, -6.9, 2.9, 1, 6.9, 2.9, 1, 6.9, -2.9, 1); 
        
    static const Mat chassisCentral_axis = (cv::Mat_<float>(8, 3) <<  
        -1.5, -16, 0, -1.5, 6, 0, 1.5, 6, 0, 1.5, -16, 0,
        -1, -14.25, 2.5, -1, 6, 2.5, 1, 6, 2.5, 1, -14.25, 2.5); 

}

#endif // TYPES_HPP