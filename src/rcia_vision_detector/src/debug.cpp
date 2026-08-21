#include "debug.hpp"
#include "angle_solver.hpp"

double get_current_timestamp() {
    auto now = chrono::steady_clock::now();

    auto duration_since_epoch = now.time_since_epoch();

    double timestamp = static_cast<double>(
        chrono::duration_cast<chrono::milliseconds>(duration_since_epoch).count());

    return timestamp;
}


auto to_round = [](double value) -> string {
    static constexpr int num_digits = 3;
    ostringstream out;
    out << fixed << setprecision(num_digits) << value;
    return out.str();
};

void getDebugInfoText(vector<string>& debug_info_text, const struct rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr, const string& aim_mode, const int& detection_count_, const double& flight_time, const string& gimbal, const double& fps, rcia_vision_interfaces::msg::GimbalCurrentInfo& gimbal_current_msg, rcia_vision_interfaces::msg::ArmorIdentifyInfo& armor_identify_msg) {
    debug_info_text.clear();

    debug_info_text.emplace_back("[0] [Now] Pitch: " + to_string(gimbal_current_msg.pitch_angle) + " Yaw: " + to_string(gimbal_current_msg.yaw_angle));
    debug_info_text.emplace_back("[1] [Set] Pitch: " + to_string(gimbal_current_msg.set_pitch_angle) + " Yaw: " + to_string(gimbal_current_msg.set_yaw_angle));
    debug_info_text.emplace_back("[2] [Mode] " + aim_mode + "");
    debug_info_text.emplace_back("[5] [center] |" + to_string(identify_info_ptr->armor_center_x) + " |" + to_string(identify_info_ptr->armor_center_y));
    debug_info_text.emplace_back("[6] Dx: " + to_round(gimbal_current_msg.diff_yaw) + " | Dy: " + to_round(gimbal_current_msg.diff_pitch) + " | Distance: " + to_string(armor_identify_msg.armor_distance));
    debug_info_text.emplace_back("[7] ArmorID: " + to_string(armor_identify_msg.armor_pattern_idx) + " | armorAcc: " + to_round(armor_identify_msg.armor_pattern_acc * 100.0) + "% | detectCount: " + to_string(detection_count_));

    debug_info_text.emplace_back("[8] flightTime: " + to_round(flight_time) + " | RH: " + to_string(armor_identify_msg.compen_height));

    debug_info_text.emplace_back("[10] [Armor] Pitch: " + to_round(armor_identify_msg.euler.x));
    debug_info_text.emplace_back("[11] [Armor] Yaw: " + to_round(armor_identify_msg.euler.y));
    debug_info_text.emplace_back("[12] [Armor] Roll: " + to_round(armor_identify_msg.euler.z));

    debug_info_text.emplace_back("[14] " + gimbal);
    debug_info_text.emplace_back("[15] " + to_string(fps));

}



void debugPutText(const Mat& draw_img, const vector<string>& debug_info_text, const int& image_height, const int& image_width) {
    int thickness = 3; double Bfont_scale = 2; double Sfont_scale = 1.75; int Binterval = 50; int Sinterval = 30;

    int Putext_x = 0, Putext_y = 0;

    for (string key : debug_info_text) {
        string index = key.substr(key.find("[") + 1, key.find("]") - key.find("[") - 1);
        string text = key.substr(key.find("]") + 1);

        if (index == "0" || index == "1" || index == "2") {
            putText(draw_img, text, Point(5, (stoi(index) + 1) * Binterval), FONT_HERSHEY_PLAIN, Bfont_scale, Scalar(255, 255, 255), thickness);
        }
        else if (index == "5") {
            vector<string> content;
            istringstream iss(text);
            for (string token; getline(iss, token, '|');) {
                content.emplace_back(token);
            }
            Putext_x = stoi(content[1]) - 300;
            Putext_y = stoi(content[2]) - 50;
        }
        else if (index == "14") {
            if (text == " blackHead") {
                putText(draw_img, text, Point(image_width - 375, 75), FONT_HERSHEY_SIMPLEX, 2, Scalar(50, 75, 255), 5);
            }
            else if (text == " whiteHead") {
                putText(draw_img, text, Point(image_width - 375, 75), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 125, 50), 5);
            }
        }

        else if (index == "15") {
            putText(draw_img, "fps:" + text, Point(35, image_height - 35), FONT_HERSHEY_SIMPLEX, 1.5, Scalar(50, 255, 50), 5);
        }

        if (Putext_x != -300 && Putext_y != 100) {
            if (index == "6" || index == "7" || index == "8" || index == "9" || index == "10" || index == "11"  || index == "12") {
                putText(draw_img, text, Point(Putext_x, Putext_y + Sinterval * stoi(index)), FONT_HERSHEY_PLAIN, Sfont_scale, Scalar(255, 255, 255), thickness);
            }
        }
    }
}


void debug_draw_image(Mat& draw_img, const vector<int>& aim_target_points, const string& gimbal, const int& image_height, const int& image_width) {
    if (gimbal == "blackHead") {
        line(draw_img, Point(image_width / 2, 0), Point(image_width / 2, image_height), Scalar(50, 50, 255), 3);
        line(draw_img, Point(0, image_height / 2), Point(image_width, image_height / 2), Scalar(50, 50, 255), 3);
    }
    else if (gimbal == "whiteHead") {
        line(draw_img, Point(image_width / 2, 0), Point(image_width / 2, image_height), Scalar(250, 230, 50), 3);
        line(draw_img, Point(0, image_height / 2), Point(image_width, image_height / 2), Scalar(250, 230, 50), 3);
    }

    if (!aim_target_points.empty()) {
        double center_x = aim_target_points[0];
        double center_y = aim_target_points[1];
        double prediction_x = aim_target_points[2];
        double prediction_y = aim_target_points[3];

        // cout << "prediction_x "  << prediction_x << "\n"
        //     << "prediction_y " << prediction_y << endl;

        if (gimbal == "blackHead") {
            circle(draw_img, Point(center_x, center_y), 5, Scalar(0, 230, 0), 3);
            circle(draw_img, Point(prediction_x, prediction_y), 10, Scalar(20, 245, 245), 3);
        }
        else if (gimbal == "whiteHead") {
            circle(draw_img, Point(center_x, center_y), 5, Scalar(0, 230, 0), 3);
            circle(draw_img, Point(prediction_x, prediction_y), 10, Scalar(225, 75, 125), 3);
        }
    }
}


cv::Mat eulerToRotationVector(double pitch_rad, double roll_rad, double yaw_rad) {
    // 创建旋转矩阵
    cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                   0, cos(pitch_rad), -sin(pitch_rad),
                   0, sin(pitch_rad), cos(pitch_rad));

    cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(yaw_rad), 0, sin(yaw_rad),
                   0, 1, 0,
                   -sin(yaw_rad), 0, cos(yaw_rad));

    cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(roll_rad), -sin(roll_rad), 0,
                   sin(roll_rad), cos(roll_rad), 0,
                   0, 0, 1);

    cv::Mat R = R_z * R_y * R_x;        // 应用欧拉角顺序

    // 从旋转矩阵中提取旋转向量
    cv::Mat_<double> rotation_vector(3, 1);
    cv::Rodrigues(R, rotation_vector);

    return rotation_vector;
}

cv::Mat getArmorRvec(const double& id, const double& tmp_yaw) {
    const static float TARGET_PITCH_RAD = 0.2618;

    double pitch_rad = id == 6 ? TARGET_PITCH_RAD : -TARGET_PITCH_RAD;              // Euler pitch_rad
    double yaw_rad = tmp_yaw;                                                       // Euler yaw_rad
    double roll_rad = 0.0;                                                          // Euler roll_rad
    
    cv::Mat rotation_vector = eulerToRotationVector(pitch_rad, roll_rad, yaw_rad);
    
    return rotation_vector;
}

void getEulerAngles(const cv::Mat& rotation_vector, rcia_vision_interfaces::msg::ArmorIdentifyInfo& armor_identify_msg) {
    armor_identify_msg.euler.x = rotation_vector.at<double>(0, 0) * (180.0 / M_PI);
    armor_identify_msg.euler.y = rotation_vector.at<double>(1, 0) * (180.0 / M_PI);
    armor_identify_msg.euler.z = rotation_vector.at<double>(2, 0) * (180.0 / M_PI);
}

/**
 * @brief 在图像上绘制目标姿态预测结果
 * @param draw_img 输出图像（绘制结果将直接修改此矩阵）
 * @param target_pose_info_ 输入目标姿态信息，包含：
 *   - armors_count: 装甲板数量
 *   - yaw: 基准偏航角
 *   - r1/r2: 装甲板旋转半径（r1为当前组，r2为交替组）
 *   - dz: 垂直方向偏移量
 *   - pose.position: 目标中心点三维坐标
 *   - id: 目标类型标识
 * 
 * 处理流程：
 * 1. 遍历所有装甲板：根据装甲板数量计算每个装甲板的方位角
 * 2. 参数选择：
 *   - 4装甲板模式：交替使用r1/r2和不同高度(z坐标)
 *   - 其他数量：统一使用r1和固定高度
 * 3. 坐标计算：
 *   - 基于极坐标公式计算相对位置
 *   - 应用坐标轴转换补偿（ROS坐标系->OpenCV坐标系）
 * 4. 三维投影：将计算出的三维坐标投影到图像平面
 * 5. 绘制预测装甲板轮廓
 * 
 * @note 坐标系转换说明：
 * - ROS坐标系 (x-forward, y-left, z-up)
 * - OpenCV坐标系 (x-right, y-down, z-forward)
 * - 计算时通过坐标取反和轴序调整实现坐标系转换
 */
void debug_target_pose(Mat& draw_img, const rcia::vision_identify::TargetSpinTopStruct& target_pose_info_, rcia_vision_interfaces::msg::ArmorIdentifyInfo& armor_identify_msg) {
    bool is_current_pair = true;                    
    size_t a_n = target_pose_info_.armors_count;    // 装甲板总数
    geometry_msgs::msg::Point p_a;                  // 单个装甲板坐标容器
    double r = 0;

    if (target_pose_info_.pose.position.x > 0.0) {
        Mat tvec, rvec;
        tvec = (cv::Mat_<double>(3, 1) << -target_pose_info_.pose.position.y, -target_pose_info_.pose.position.z, target_pose_info_.pose.position.x);
        rvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
        // 三维坐标投影到图像平面
        vector<cv::Point2f> imgpts;
        projectPoints(
            Reprojection::central_axis,         // 装甲板3D特征点
            rvec, tvec,                         // 旋转向量和平移向量
            PNP::black_camera_matrix,           // 相机内参矩阵
            PNP::black_dist_coeffs,             // 畸变系数
            imgpts);                            // 输出投影后的2D点集
        reproject_and_draw(draw_img, imgpts, "chassisCentral");
    }
    
    // 遍历所有装甲板生成预测位置
    for (size_t i = 0; i < a_n; i++) {
        double tmp_yaw = target_pose_info_.yaw + i * (2 * M_PI / a_n);
        if (a_n == 4) {
            r = is_current_pair ? target_pose_info_.r1 : target_pose_info_.r2;
            p_a.z = target_pose_info_.pose.position.z + (is_current_pair ? 0 : target_pose_info_.dz);
            is_current_pair = !is_current_pair;
        }
        else {
            r = target_pose_info_.r1;
            p_a.z = target_pose_info_.pose.position.z;
        }
        
        // 计算装甲板相对位置（极坐标公式）
        // 注：因ROS坐标系方向定义，需取反Y轴坐标
        p_a.x = target_pose_info_.pose.position.x - r * cos(tmp_yaw);
        p_a.y = -target_pose_info_.pose.position.y - r * sin(tmp_yaw);

        // 坐标系转换：ROS->OpenCV 
        // (ROS: x-forward,y-left,z-up -> OpenCV: x-right,y-down,z-forward)
        Mat tvec, rvec;
        tvec = (cv::Mat_<double>(3, 1) << p_a.y, -p_a.z, p_a.x);
        rvec = getArmorRvec(target_pose_info_.id, tmp_yaw);

        // 三维坐标投影到图像平面
        vector<cv::Point2f> imgpts;
        projectPoints(
            Reprojection::armor_axis,       // 装甲板3D特征点
            rvec, tvec,                     // 旋转向量和平移向量
            PNP::black_camera_matrix,       // 相机内参矩阵
            PNP::black_dist_coeffs,          // 畸变系数
            imgpts);                        // 输出投影后的2D点集

        // 在图像上绘制预测装甲板轮廓
        if (i == 0) {
            getEulerAngles(rvec, armor_identify_msg);
            reproject_and_draw(draw_img, imgpts, "currArmor");
        }else{
            reproject_and_draw(draw_img, imgpts, "predArmor");
        }
    }
}


void reproject_and_draw(cv::Mat &draw_img, std::vector<cv::Point2f> imgpts, std::string draw_type) {
    if (draw_type == "currArmor") {
        cv::drawContours(draw_img, vector<vector<Point>>{{imgpts[4], imgpts[5], imgpts[6], imgpts[7]}}, 0, cv::Scalar(255, 125, 50), 3);
    }

    else if (draw_type == "predArmor") {
        cv::drawContours(draw_img, vector<vector<Point>>{{imgpts[0], imgpts[1], imgpts[2], imgpts[3]}}, 0, cv::Scalar(50, 255, 50), 3);
        std::vector<std::vector<cv::Point>> contours{{imgpts[4], imgpts[5], imgpts[6], imgpts[7]}};
    }

    else if (draw_type == "chassisCentral") {
        cv::drawContours(draw_img, vector<vector<Point>>{{imgpts[0], imgpts[1], imgpts[2], imgpts[3]}}, -1, cv::Scalar(255, 255, 255), -10);
        cv::drawContours(draw_img, vector<vector<Point>>{{imgpts[4], imgpts[5], imgpts[6], imgpts[7]}}, -1, cv::Scalar(250, 50, 150), -10);
    }

    else if (draw_type == "fire_chassisCentral") {
        cv::drawContours(draw_img, vector<vector<Point>>{{imgpts[0], imgpts[1], imgpts[2], imgpts[3]}}, -1, cv::Scalar(50, 50, 250), -10);
        cv::drawContours(draw_img, vector<vector<Point>>{{imgpts[4], imgpts[5], imgpts[6], imgpts[7]}}, -1, cv::Scalar(10, 10, 10), -10);
    }
}