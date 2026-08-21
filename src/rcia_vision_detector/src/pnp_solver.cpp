#include "vision_initialize.hpp"

#include "pnp_solver.hpp"
#include "angle_solver.hpp"

#include "Types.hpp"

// PnP解算
void solve_pnp(
    const struct rcia::vision_identify::GimbalInfoStruct::CameraInfoStruct& camera_info,
    Mat &armor_rvec,
    Mat &armor_tvec,
    rcia_vision_interfaces::msg::ArmorIdentifyInfo& armor_identify_msg)
{
    cv::Mat rotM;
    try {
        vector<Point2f> object_2d_point;
        
        for (const auto &p2d : armor_identify_msg.armor_2d_points) {
            object_2d_point.emplace_back(Point2f(p2d.x, p2d.y));
        }

        // 根据装甲板类型选择 3D 点
        const vector<Point3f> &object_3d_points = (armor_identify_msg.armor_type == "small_armor") 
            ? PNP::object_3d_points_small 
            : PNP::object_3d_points_large;

        // 求解 PnP
        bool found = solvePnP(object_3d_points, object_2d_point, camera_info.camera_matrix, 
                              camera_info.dist_coeffs, armor_rvec, armor_tvec, false, SOLVEPNP_IPPE);

        if (!found) cerr << "Error: PnP problem not solved!" << endl;
        
        Rodrigues(armor_rvec, rotM);

        // 将 armor_tvec 转换为 geometry_msgs::msg::Point32
        armor_identify_msg.pnp_armor_tvec.x = armor_tvec.at<double>(0);
        armor_identify_msg.pnp_armor_tvec.y = armor_tvec.at<double>(1);
        armor_identify_msg.pnp_armor_tvec.z = armor_tvec.at<double>(2);

        // 将 armor_rvec 转换为 geometry_msgs::msg::Point32
        armor_identify_msg.pnp_armor_rvec.x = armor_rvec.at<double>(0);
        armor_identify_msg.pnp_armor_rvec.y = armor_rvec.at<double>(1);
        armor_identify_msg.pnp_armor_rvec.z = armor_rvec.at<double>(2);

        // 计算俯仰角（pnp_armor_pitch）
        armor_identify_msg.euler.x = atan2(rotM.at<double>(2, 1), rotM.at<double>(2, 2)) * (180.0 / CV_PI);
    
        if ((rotM.at<double>(2, 1) * rotM.at<double>(2, 1) + rotM.at<double>(2, 2)) < 0) {
            rotM.at<double>(2, 2) = 0;
        }
        // 计算偏航角（pnp_armor_yaw）
        armor_identify_msg.euler.y = atan2(-rotM.at<double>(2, 0), 
                                              sqrt(rotM.at<double>(2, 1) * rotM.at<double>(2, 1) + rotM.at<double>(2, 2)) * rotM.at<double>(2, 2)) * (180.0 / CV_PI);
    
        // 计算滚转角（pnp_armor_roll）
        armor_identify_msg.euler.z = atan2(rotM.at<double>(1, 0), rotM.at<double>(0, 0)) * (180.0 / CV_PI);


    }

    catch (const cv::Exception &e) {
        cerr << "OpenCV Exception in solve_pnp: " << e.what() << endl;
    }
    catch (const exception &e) {
        cerr << "Exception in solve_pnp: " << e.what() << endl;
    }
}

void angle_solver(rcia_vision_interfaces::msg::GimbalCurrentInfo& gimbal_current_msg,
                  rcia_vision_interfaces::msg::ArmorIdentifyInfo& armor_identify_msg)
{
    double x = armor_identify_msg.pnp_armor_tvec.x;
    double y = armor_identify_msg.pnp_armor_tvec.y;
    double z = armor_identify_msg.pnp_armor_tvec.z;

    double pitchRelativeAngle = -atan2(y, z) * (180.0 / M_PI);
    double yawRelativeAngle = -atan2(x, z) * (180.0 / M_PI);

    gimbal_current_msg.diff_yaw = yawRelativeAngle;

    gimbal_current_msg.diff_pitch = pitchRelativeAngle;
}


void point_solver(pair<double, double> angle_degree,
    const struct rcia::vision_identify::GimbalInfoStruct::CameraInfoStruct& camera_info,
    struct rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr)
{
    double x2_rad = tan(angle_degree.first * PI / 180.0);

    double y2_rad = tan(angle_degree.second * PI / 180.0);

    double k1 = camera_info.dist_coeffs.at<double>(0, 0);
    double k2 = camera_info.dist_coeffs.at<double>(0, 1);
    double p1 = camera_info.dist_coeffs.at<double>(0, 2);
    double p2 = camera_info.dist_coeffs.at<double>(0, 3);

    double x1 = x2_rad / (1 + k1 * x2_rad * x2_rad + k2 * x2_rad * x2_rad * x2_rad * x2_rad);

    double y1 = y2_rad / (1 + k1 * y2_rad * y2_rad + k2 * y2_rad * y2_rad * y2_rad * y2_rad);

    double x0 = x1 / (1 + p1 * (x1 * x1 + y1 * y1)) - p2 * (x1 * x1 + y1 * y1);

    double y0 = y1 / (1 + p2 * (x1 * x1 + y1 * y1)) - p1 * (x1 * x1 + y1 * y1);

    identify_info_ptr->armor_prediction_x = round(x0 * camera_info.camera_matrix.at<double>(0, 0) + camera_info.camera_matrix.at<double>(0, 2));

    identify_info_ptr->armor_prediction_y = round(y0 * camera_info.camera_matrix.at<double>(1, 1) + camera_info.camera_matrix.at<double>(1, 2));
}
