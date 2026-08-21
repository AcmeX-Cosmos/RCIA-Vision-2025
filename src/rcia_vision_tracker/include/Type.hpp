#ifndef TYPES_HPP
#define TYPES_HPP

#include <chrono>
#include <iostream>
#include <string>
#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>
#include <angles/angles.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

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
#include "rcia_vision_interfaces/msg/serial_receive_data.hpp"
#include "rcia_vision_interfaces/msg/serial_transmit_data.hpp"
#include "rcia_vision_interfaces/msg/armor_bapose_info.hpp"
#include "rcia_vision_interfaces/msg/target_spin_top.hpp"
#include "rcia_vision_interfaces/msg/odom_measurement.hpp"
#include "rcia_vision_interfaces/msg/tracker_state.hpp"


//using namespace
using namespace std;
using namespace cv;
using namespace Eigen;

namespace rcia::spinTop_predictor {
}


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

}


#endif // TYPES_HPP