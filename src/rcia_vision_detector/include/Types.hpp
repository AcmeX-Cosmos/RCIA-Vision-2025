#ifndef TYPES_HPP
#define TYPES_HPP

#include "vision_initialize.hpp"

// ROS
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// ROS2 msg
#include "rcia_vision_interfaces/msg/gimbal_current_info.hpp"
#include "rcia_vision_interfaces/msg/gimbal_previous_info.hpp"
#include "rcia_vision_interfaces/msg/armor_identify_info.hpp"
#include "rcia_vision_interfaces/msg/tracker_state.hpp"
#include "rcia_vision_interfaces/msg/target_spin_top.hpp"
#include "rcia_vision_interfaces/msg/serial_receive_data.hpp"

namespace rcia::SerialProtocol {
    struct SerialDataStruct {
        int pitch_angle;
        int yaw_angle;

        string gimbal = "blackHead";

        int bullet_speed = 26;
        double fps = 0.0;
        string enemy_color;

        uint8_t gyroScopeMode;
    };
}


namespace rcia::vision_identify {
    struct IdentifyInfoStruct {
        int armor_center_x = 0;
        int armor_center_y = 0;
    
        int armor_prediction_x = 0;
        int armor_prediction_y = 0;
        
        int detector_center_x = 0;
        int detector_center_y = 0;
    
        double armor_width = 0;
        double armor_height = 0;
        // double armor_area = 0;
    };
    
    // 云台信息
    struct GimbalInfoStruct {
        // 相机信息
        struct CameraInfoStruct {
            bool init_flag = false;
            string gimbal_name;
            Mat camera_matrix;
            Mat dist_coeffs;
        };

        CameraInfoStruct camera_info;
    };

    struct TargetSpinTopStruct {
        bool tracking;
        uint8_t id;
        uint8_t armors_count;
        geometry_msgs::msg::Pose pose;
        double yaw;
        float r1;
        float r2;
        float dz;
    };
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
    constexpr double small_half_x = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;  // 半宽
    constexpr double small_half_y = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0; // 半高
    // 计算大装甲板的半宽和半高（单位：米）
    constexpr double large_half_x = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;  // 半宽
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
    // 单位(m)
    static const Mat armor_axis = (cv::Mat_<float>(8, 3) << 
        -0.069, -0.029, 0.000, -0.069, 0.029, 0.000, 0.069, 0.029, 0.000, 0.069, -0.029, 0.000,
        -0.069, -0.029, 0.010, -0.069, 0.029, 0.010, 0.069, 0.029, 0.010, 0.069, -0.029, 0.010); 

    static const Mat central_axis = (cv::Mat_<float>(8, 3) <<  
        -0.005, -0.050, 0.000, -0.005, 0.050, 0.000, 0.005, 0.050, 0.000, 0.005, -0.050, 0.000,
        -0.005, -0.050, 0.010, -0.005, 0.050, 0.010, 0.005, 0.050, 0.010, 0.005, -0.050, 0.010);
    
    static const cv::Mat chassis_axis = (cv::Mat_<float>(2, 3) <<
        -0.400, -0.150, 0.000,  0.400,  0.150, 0.000);

}


#endif // TYPES_HPP