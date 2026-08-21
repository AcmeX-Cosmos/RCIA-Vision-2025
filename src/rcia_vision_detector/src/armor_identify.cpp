#include "vision_initialize.hpp"

#include "armor_identify.hpp"
#include "angle_solver.hpp"

#include "Types.hpp"

namespace rcia::vision_identify {
/**
 * @brief 装甲识别系统构造函数
 * @param node ROS2节点共享指针
 * 
 * 初始化内容包括：
 * - ROS图像传输接口
 * - 算法组件（卡尔曼滤波、装甲分析器）
 * - 状态控制参数
 * - 图像处理参数
 * - 调试配置
 * 
 * @note 需确保node参数已完成初始化
 */
ArmorIdentifier::ArmorIdentifier(const rclcpp::NodeOptions &options)
    : rclcpp::Node("rcia_vision_detector", options), // 传递options给基类
        buffer1_{}, buffer2_{}
    {
        // 是否显示调试信息
        isDebugModeEnabled_ = this->declare_parameter("debug_mode_enabled", false);
        this->declare_parameter("camera_matrix.data", 
        std::vector<double>{1557.112832049196, 0, 640.0984500155263, 
                            0, 1735.09586187522, 512.0025696784391,
                            0.0, 0.0, 1.0});
        this->declare_parameter("dist_coeffs.data", 
        std::vector<double>{0.02379281673136642,  -2.931918199663403, -0.0001751467257614772, 0.001130183050415235,  42.30457985450274});

        is_initialized = true;
        last_armor_height = 0; last_armor_width = 0;
        detector_center_x = 0; detector_center_y = 0;
        last_center_x = 0; last_center_y = 0;
        last_compensation_height = 0;
        frame_count_ = 0; fps_ = 0.0;
        last_time_ = std::chrono::high_resolution_clock::now();
        current_read_buffer_ = &buffer1_;   // 初始指向buffer1

        vector<int> aim_target_points;

        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        armor_identify_info_pub_ = this->create_publisher<rcia_vision_interfaces::msg::ArmorIdentifyInfo>("armor_identify_info", 2);
        gimbal_current_info_pub_ = this->create_publisher<rcia_vision_interfaces::msg::GimbalCurrentInfo>("gimbal_current_info", 2);

        // 订阅图像话题
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", rclcpp::SensorDataQoS(),
            std::bind(&ArmorIdentifier::imageCallback, this, std::placeholders::_1));

        serial_sub_ = create_subscription<rcia_vision_interfaces::msg::SerialReceiveData>(
            "electrl_data",
            rclcpp::SensorDataQoS().keep_last(1).best_effort(),
            [this](const rcia_vision_interfaces::msg::SerialReceiveData::SharedPtr msg)
            {
                // 获取非当前读的缓冲区进行写入
                auto *write_buffer =
                    (current_read_buffer_.load() == &buffer1_) ? &buffer2_ : &buffer1_;
    
                // 更新缓冲区数据
                write_buffer->pitch_angle = msg->pitch_angle;
                write_buffer->yaw_angle = msg->yaw_angle;
                write_buffer->bullet_speed = msg->bullet_speed;
    
                std::string mecolor = "blue";
                write_buffer->enemy_color = (mecolor == "blue") ? "red" : "blue";
    
                // 原子切换读缓冲区指针
                current_read_buffer_.store(write_buffer);
            });

        tracker_state_sub_ = this->create_subscription<rcia_vision_interfaces::msg::TrackerState>(
            "tracker_state", 2, std::bind(&ArmorIdentifier::tracker_state_callback, this, std::placeholders::_1));

        target_info_sub_ = this->create_subscription<rcia_vision_interfaces::msg::TargetSpinTop>(
            "spin_top_topic", 2, std::bind(&ArmorIdentifier::target_info_callback, this, std::placeholders::_1));

        armor_detector_ = init_ArmorDetector();

        static bool opencv_useOptimized = [](){
            cv::setUseOptimized(true);
            cv::setNumThreads(2);           // 此处线程并非越多越好
            // 打印优化状态
            std::cout << "OpenCV优化状态: " << cv::useOptimized() << "\n";
            std::cout << "当前线程数: " << cv::getNumThreads() << "\n";
            std::cout << "IPP 状态: " << cv::ipp::useIPP() << "\n";     // 验证 IPP 是否启用（OpenCV 4.5 及以上版本）

            return true;
        }();
    }

std::unique_ptr<ArmorDetector> ArmorIdentifier::init_ArmorDetector()
{
    ArmorDetector::DetectorParams detector_params = {
        .enemy_color = declare_parameter("enemy_color", "red"),
        .red_thresh = static_cast<int>(declare_parameter("red_thresh", 100)),
        .blue_thresh = static_cast<int>(declare_parameter("blue_thresh", 50)),
        .tolerance = declare_parameter("tolerance", 1.25),
        .max_height_ratio = declare_parameter("max_height_ratio", 1.5),
        .min_armor_ratio = declare_parameter("min_armor_ratio", 1.5),
        .max_armor_ratio = declare_parameter("max_armor_ratio", 8.0),
        .armor_type_thresh = declare_parameter("armor_type_thresh", 3.6),
        .angle_diff_thresh = declare_parameter("angle_diff_thresh", 7.5),
        .min_contour_area = declare_parameter("min_contour_area", 5.0),
        .area_valid_ratio = declare_parameter("area_valid_ratio", 0.5),
        .max_aspect_ratio = declare_parameter("max_aspect_ratio", 2.5),
        .max_contours = static_cast<int>(declare_parameter("max_contours", 20))};

    auto armor_detector = std::make_unique<ArmorDetector>(detector_params);

    set_paramters_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &ArmorIdentifier::on_set_parameters, this, std::placeholders::_1));

    return armor_detector;
}

rcl_interfaces::msg::SetParametersResult ArmorIdentifier::on_set_parameters(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters)
    {   
        if (param.get_name() == "enemy_color")              armor_detector_->detector_params.enemy_color = param.as_string();
        else if (param.get_name() == "red_thresh")          armor_detector_->detector_params.red_thresh = param.as_int();
        else if (param.get_name() == "blue_thresh")         armor_detector_->detector_params.blue_thresh = param.as_int();
        else if (param.get_name() == "tolerance")           armor_detector_->detector_params.tolerance = param.as_double();
        else if (param.get_name() == "max_height_ratio")    armor_detector_->detector_params.max_height_ratio = param.as_double();
        else if (param.get_name() == "min_armor_ratio")     armor_detector_->detector_params.min_armor_ratio = param.as_double();
        else if (param.get_name() == "max_armor_ratio")     armor_detector_->detector_params.max_armor_ratio = param.as_double();
        else if (param.get_name() == "armor_type_thresh")   armor_detector_->detector_params.armor_type_thresh = param.as_double();
        else if (param.get_name() == "angle_diff_thresh")   armor_detector_->detector_params.angle_diff_thresh = param.as_double();
        else if (param.get_name() == "min_contour_area")    armor_detector_->detector_params.min_contour_area = param.as_double();
        else if (param.get_name() == "area_valid_ratio")    armor_detector_->detector_params.area_valid_ratio = param.as_double();
        else if (param.get_name() == "max_aspect_ratio")    armor_detector_->detector_params.max_aspect_ratio = param.as_double();
        else if (param.get_name() == "max_contours")        armor_detector_->detector_params.max_contours = param.as_int();
    }
    return result;
}

void ArmorIdentifier::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    try
    {
        auto src_image = cv_bridge::toCvShare(img_msg, "bgr8")->image;

        // 检查Mat是否有效
        if (src_image.empty())
        {
            RCLCPP_ERROR(get_logger(), "OpenCV Mat为空，数据解析失败");
            return;
        }
        // 计算帧率
        auto now = std::chrono::high_resolution_clock::now();
        frame_count_++;
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time_).count();
        // 每秒更新一次
        if (duration >= 1000) { 
            fps_ = frame_count_ * 1000.0 / duration;
            frame_count_ = 0;
            last_time_ = now;
            // RCLCPP_INFO(get_logger(), "Current FPS: %.2f", fps_);
            // std::cout << "Current FPS: " << fps_ << std::endl;
        }

        auto *read_buffer = current_read_buffer_.load(std::memory_order_acquire);
        read_buffer->enemy_color = "red";
        read_buffer->fps = fps_;

        // 主识别程序
        identify_armor(src_image, read_buffer);
    }
    catch (const cv::Exception &e)
    {
        RCLCPP_ERROR(get_logger(), "OpenCV异常: %s", e.what());
    }
}




/**
 * @brief 执行目标姿态坐标变换
 * @param[in] target_info_msg 输入目标状态信息（包含header/position/yaw等）
 * @param[out] target_pose_info_ 输出转换后的目标姿态信息
 * 
 * 处理流程：
 * 1. 初始化位姿数据：继承消息头，设置初始位置和零偏转RPY四元数
 * 2. 坐标系转换：将目标位姿转换到相机坐标系
 * 3. 解算旋转矩阵：从变换后四元数提取yaw角（忽略roll/pitch）
 * 4. 后处理：校正yaw方向符号，记录半径和高度参数
 * 
 * @note 因前期坐标变换的取反操作，最终需对计算结果取反保持方向一致性
 */
void ArmorIdentifier::target_pose_transform(const std::shared_ptr<rcia_vision_interfaces::msg::TargetSpinTop> target_info_msg, TargetSpinTopStruct& target_pose_info_) {
    geometry_msgs::msg::PoseStamped pose_in;
    geometry_msgs::msg::PoseStamped pose_out;
    
    pose_in.header = target_info_msg->header;
    pose_in.pose.position = target_info_msg->position;

    tf2::Quaternion q_in;
    //      roll    pitch    yaw     
    q_in.setRPY(0.0, 0.0, degreesToRadians(target_info_msg->yaw));
    pose_in.pose.orientation = tf2::toMsg(q_in);

    auto transform = tf2_buffer_->lookupTransform("camera_link", pose_in.header.frame_id, tf2::TimePointZero); // 使用最新可用变换

    tf2::doTransform(pose_in, pose_out, transform);
    
    target_pose_info_.tracking = target_info_msg->tracking;
    target_pose_info_.armors_count = target_info_msg->armors_count;
    target_pose_info_.id = target_info_msg->id;

    target_pose_info_.pose.position.x = pose_out.pose.position.x;
    target_pose_info_.pose.position.y = pose_out.pose.position.y;
    target_pose_info_.pose.position.z = pose_out.pose.position.z;

    tf2::Quaternion q_out;
    tf2::fromMsg(pose_out.pose.orientation, q_out);
    tf2::Matrix3x3 rotation_matrix(q_out);
    
    [[maybe_unused]] double _unusedroll, _unusedpitch;
    rotation_matrix.getRPY(_unusedroll, _unusedpitch, target_pose_info_.yaw);

    // 根据前期坐标变换的取反操作调整yaw轴符号
    target_pose_info_.yaw = -target_pose_info_.yaw;
    target_pose_info_.r1 = target_info_msg->r1;
    target_pose_info_.r2 = target_info_msg->r2;
    target_pose_info_.dz = target_info_msg->dz;

    // decide_range use
    this->target_central_tvec = (cv::Mat_<double>(3, 1) << -target_pose_info_.pose.position.y, -target_pose_info_.pose.position.z, target_pose_info_.pose.position.x);

}
    
void ArmorIdentifier::target_info_callback(const std::shared_ptr<rcia_vision_interfaces::msg::TargetSpinTop> target_info_msg) {
    target_pose_transform(target_info_msg, this->target_pose_info_);
}

void ArmorIdentifier::tracker_state_callback(const std::shared_ptr<rcia_vision_interfaces::msg::TrackerState> tracker_state_msg) {
    int max_detection_count = 50;
    this->tracker_state_ = tracker_state_msg->state;

    if (this->tracker_state_ == "Patrol" && tracker_state_msg->detection_count == 0) {
        this->is_initialized = true;
    }
    else if (this->tracker_state_ == "Tracking") {
        this->detection_count_ = max_detection_count;
    }
    else if (this->tracker_state_ == "Detecting") {
    }
    else if (this->tracker_state_ == "TempLost") {
        this->detection_count_ = max_detection_count - tracker_state_msg->lost_count;
        this->detection_count_ = max(this->detection_count_, 0);
    }
    
    this->flight_time = tracker_state_msg->flight_time;
    
    this->gimbal_current_msg.set_pitch_angle = tracker_state_msg->last_pitch;
    this->gimbal_current_msg.set_yaw_angle = tracker_state_msg->last_yaw;
    
    this->armor_identify_msg.armor_distance = tracker_state_msg->armor_distance;
    this->armor_identify_msg.compen_height = tracker_state_msg->compen_height;
}


/**
 * 从候选装甲板中筛选最优目标
 * @param[in] candidate_armors 候选装甲板集合（轮廓+类型）
 * @param[out] best_contour 输出最优装甲板轮廓
 * @param[out] armor_type 输出装甲板类型
 * 
 * 算法流程：
 * 1. 遍历所有候选装甲板
 * 2. 坐标转换：将相对坐标转换为绝对坐标
 * 3. 模式识别：验证装甲板有效性
 * 4. 距离计算：选择最接近图像中心的装甲板
 */
void ArmorIdentifier::select_optimal_armor(
    vector<pair<vector<Point>, string>> &armor_candidates_)
{
    try
    {
        unordered_map<int, double> distance_records;
        bool found_flag = false;

        for (int light_index = 0; light_index < armor_candidates_.size(); ++light_index)
        {
            vector<Point> current_armor_contour = armor_candidates_[light_index].first;
            
            // 添加偏移量 
            for (auto &point : current_armor_contour) {
                point.x += this->detector_start_x;
                point.y += this->detector_start_y;
            }

            Rect armorRect = boundingRect(current_armor_contour);
            int centerX = armorRect.x + armorRect.width * 0.5;
            int centerY = armorRect.y + armorRect.height * 0.5;

            // cout << centerX << " " << centerY << endl;

            string armor_type = armor_candidates_[light_index].second;

            int armorPatternIdx;
            double armorPatternAcc;
            ptClassifier.getSimpleRoiPattern(current_armor_contour, draw_img, armor_type, this->detection_count_, armorPatternIdx, armorPatternAcc);
            // int armorPatternIdx = 1;
            // double armorPatternAcc = 0.99;
            
            // cout << "armorPatternIdx:" << armorPatternIdx << "\t";
            // cout << "armorPatternAcc:" << armorPatternAcc << endl;
            
            if (armorPatternIdx == 0 || armorPatternIdx == 9) {
                continue;
            }
            else {
                this->armorIdx = armorPatternIdx;
                this->armorAcc = armorPatternAcc;
            }

            double dist = sqrt(pow(image_width / 2 - centerX, 2) + pow(image_height / 2 - centerY, 2));
            distance_records[light_index] = dist;

            // 将决策范围偏移值 赋值回去
            armor_candidates_[light_index].first = current_armor_contour;
            found_flag = true;
        }

        
        // 此处可通过 ArmorIdentifyInfo[] 添加多块装甲板
        if (found_flag) {
            auto mindis_index = min_element(distance_records.begin(), distance_records.end(),
                [](const auto& a, const auto& b) { return a.second < b.second; });

            auto armor_info = move(armor_candidates_[mindis_index->first]);
            
            for (const auto &point : armor_info.first) {
                geometry_msgs::msg::Point32 p2d;
                p2d.x = point.x;
                p2d.y = point.y;
                p2d.z = 0.0;
                armor_identify_msg.armor_2d_points.emplace_back(p2d);

                // circle(this->draw_img, point, 1, Scalar(0, 255, 0), -1);
            }
            armor_identify_msg.armor_pattern_acc = this->armorAcc;
            armor_identify_msg.armor_pattern_idx = this->armorIdx;
            armor_identify_msg.armor_type = armor_info.second;
        }
    }
    catch (const exception &e){
        // 捕获异常并进行处理
        cerr << "Error occurred in select_optimal_armor function." << e.what() << endl;
    }
}

/**
 * 动态调整检测区域范围
 * @param identify_info_ptr 包含姿态信息的结构体指针
 * 
 * 算法原理：
 * 1. 首次检测时使用全图范围
 * 2. 后续基于历史检测结果动态调整ROI：
 *    - 通过装甲板尺寸计算基准缩放比例
 *    - 结合检测次数进行动态范围调整
 *    - 双限制策略：既保证最小跟踪范围，又防止范围过大
 */
void ArmorIdentifier::decide_range(Mat& src_img) {
    if (this->detection_count_ == 0) {
        this->detector_start_x = 0;
        this->detector_start_y = 0;
        this->detector_end_x = this->image_width;
        this->detector_end_y = this->image_height;
        return;
    }

    vector<Point2f> imgpts;
    projectPoints(Reprojection::chassis_axis, Mat::zeros(3,1,CV_64F), this->target_central_tvec,
                 gimbal.camera_info.camera_matrix, gimbal.camera_info.dist_coeffs, imgpts);
 
    this->detector_start_x = max(0, static_cast<int>(imgpts[0].x));
    this->detector_start_y = max(0, static_cast<int>(imgpts[0].y));
    this->detector_end_x = min(this->image_width, static_cast<int>(imgpts[1].x));
    this->detector_end_y = min(this->image_height, static_cast<int>(imgpts[1].y));

    src_img = src_img(Rect(detector_start_x, detector_start_y,
                           detector_end_x - detector_start_x,
                           detector_end_y - detector_start_y));
    
}

/**
 * 装甲板识别核心处理函数
 * @param src_img 输入图像（BGR格式）
 * @param fps 当前帧率（用于控制滤波参数）
 * @param pitch_now_angle 云台当前俯仰角
 * @param yaw_now_angle 云台当前偏航角
 * @param bullet_speed 子弹初速度（m/s）
 * @param enemy_color 敌方颜色标识（"red"/"blue"）
 * @param gyroScopeMode 是否击打陀螺标志（0x00:关闭, 0x01:开启）
 * @param gimbal_name 云台类型标识（"blackHead"/"whiteHead"）
 * @param origin_yaw_angle 开机时偏航角角度
 * @return tuple<目标坐标点, 俯仰设定角, 偏航设定角, 检测计数>
 *
 * 处理流程：
 * 1. 输入校验：检查图像有效性
 * 2. 初始化控制参数：重置角度设定值，设置云台类型
 * 3. 预处理图像：动态调整ROI区域
 * 4. 灯条检测：获取候选装甲板区域
 * 5. 目标选择：根据距离、模式验证选择最优装甲板
 * 6. 姿态解算：
 *    - 坐标系转换（像素->世界）
 *    - 飞行时间预测
 *    - 弹道补偿（重力和空气阻力）
 * 7. 运动预测：
 *    - 陀螺模式：使用角速度预测
 *    - 普通模式：卡尔曼滤波预测
 * 8. 输出控制量：计算最终的云台角度设定值
 * 9. 状态保持：存储当前识别结果供下一帧使用
 * 
 * 特殊处理：
 * - 陀螺模式下的开火条件判断（角度偏差+时间间隔）
 * - 目标丢失时的历史数据预测
 * - 检测计数动态调整ROI范围
 * 
 * 调试支持：
 * - 实时显示识别结果和预测轨迹
 * - 显示关键参数（识别置信度、飞行时间等）
 */
void ArmorIdentifier::identify_armor(
    Mat& src_img, const rcia::SerialProtocol::SerialDataStruct *serial_data_ptr)
{
    
    auto identify_info = rcia::vision_identify::IdentifyInfoStruct();
    auto* identify_info_ptr = &identify_info;
    try
    {
        _initialize_control(serial_data_ptr, identify_info_ptr);

        _initialize_identify(src_img, identify_info_ptr);

        armor_detector_->detect_armor_lights(src_img, serial_data_ptr->enemy_color, &armor_candidates_, lights);

        if (!armor_candidates_.empty()) select_optimal_armor(armor_candidates_);
        
        if (!armor_identify_msg.armor_2d_points.empty())
        {
            armor_detector_->compute_armor_info(identify_info_ptr, armor_identify_msg);

            Mat armor_rvec, armor_tvec;

            // pnp姿态解算
            pixel_to_angle(gimbal.camera_info, identify_info_ptr, armor_rvec, armor_tvec, "PnP", gimbal_current_msg, armor_identify_msg);

            armor_identify_info_pub_->publish(armor_identify_msg);
 
            predict_angle_to_pixel(gimbal.camera_info, identify_info_ptr, gimbal_current_msg);

            // gimbal_current_info_pub_->publish(gimbal_current_msg);

            // # ---------------------------------------------------------------------------------- #
            // # ----------------------------- # Start storing data # ----------------------------- #
            // # ----------------------- # Armor identification completed # ----------------------- #
            // # ------------------------- # Store this recognition data # ------------------------ #
            // # ---------# Prevent not being able to recognize the target the next time# --------- #
            // # ---------------------------------------------------------------------------------- #
            
            aim_target_points = {identify_info_ptr->armor_center_x, identify_info_ptr->armor_center_y, identify_info_ptr->armor_prediction_x, identify_info_ptr->armor_prediction_y};
            
            last_center_x = identify_info_ptr->armor_center_x;
            last_center_y = identify_info_ptr->armor_center_y;

            last_armor_width = identify_info_ptr->armor_width;
            last_armor_height = identify_info_ptr->armor_height;

            // # ---------------------------------------------------------------------------------- #
            // # ----------------------------- # Stop storing data # ------------------------------ #
            // # ---------------------------------------------------------------------------------- #
        }
        else {
            armor_identify_msg.armor_type = "null";

            armor_identify_info_pub_->publish(armor_identify_msg);

            // 重置为巡逻模式 maybe can optimizable
            if (this->detection_count_ <= 0) {
                this->is_initialized = true;
            }
        }

        if (isDebugModeEnabled_) {
            display_debug_info(identify_info_ptr, serial_data_ptr->gimbal, serial_data_ptr->fps);
        }
    }

    catch (const exception &e) {
        cerr << "Error occurred in identify_armor function." << e.what() << endl;
    }
}

void ArmorIdentifier::display_debug_info(const rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr, const string& gimbal_name, const double& fps) {
    
    debug_target_pose(this->draw_img, this->target_pose_info_, armor_identify_msg);

    debug_draw_image(this->draw_img, aim_target_points, gimbal_name, image_height, image_width);

    getDebugInfoText(debug_info_text, identify_info_ptr, this->tracker_state_, detection_count_, this->flight_time, gimbal_name, fps, gimbal_current_msg, armor_identify_msg);

    debugPutText(this->draw_img, debug_info_text, image_height, image_width);

    circle(this->draw_img, Point(this->detector_center_x, this->detector_center_y), 75, Scalar(0, 255, 255), 3);

    if (gimbal.camera_info.gimbal_name == "blackHead")
    {
        // resize(lights, lights, Size(), 0.5, 0.5);
        // imshow("Blacklights", lights);
        resize(this->draw_img, this->draw_img, Size(), 0.35, 0.35);
        imshow("blackHead", this->draw_img);
        waitKey(1);
    }
}




/**
 * @brief 初始化装甲识别流程
 * @param[in,out] img 输入图像（将被裁剪为ROI区域）
 * @param[in] identify_info_ptr 识别参数结构体指针
 * 
 * 工作流程：
 * 1. 清除上一帧的候选数据
 * 2. 复制调试用图像副本
 * 3. 根据初始化标志位：
 *    - true: 进入全图巡逻模式
 *    - false: 使用动态计算的ROI区域
 * 4. 记录ROI调试信息
 */
void ArmorIdentifier::_initialize_identify(Mat& src_img, const rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr)
{
    isDebugModeEnabled_ = this->get_parameter("debug_mode_enabled").as_bool();

    this->armor_candidates_.clear();

    this->armor_identify_msg.armor_2d_points.clear();

    src_img.copyTo(this->draw_img);

    if (this->is_initialized) {
        this->_initialize_patrol(src_img.rows, src_img.cols);
        this->is_initialized = false;
    }
    // else {
    //     this->decide_range(src_img);
    // }

    // cout << "detector_start_x" << detector_start_x << "   detector_start_y" << detector_start_y << "\ndetector_end_x" << detector_end_x << "   detector_end_y" << detector_end_y << endl;
}





/**
 * 初始化装甲识别控制参数
 * @param[in] pitch_now_angle 当前云台俯仰角（单位：度）
 * @param[in] yaw_now_angle 当前云台偏航角（单位：度） 
 * @param[in] origin_yaw_angle 初始偏航角基准值
 * @param[in] gimbal_name 云台类型标识（"blackHead"/"whiteHead"）
 * @param[out] identify_info_ptr 控制信息结构体指针
 *
 * 
 * 功能说明：
 * 1. 初始化当前姿态角度
 * 2. 重置控制输出量
 * 3. 记录云台类型配置
 * 4. 初始化开火状态标识
 * 
 * 注意事项：
 * - 需在每帧识别开始时调用
 * - 保证identify_info_ptr指针有效性
 */
void ArmorIdentifier::_initialize_control(
    const rcia::SerialProtocol::SerialDataStruct *serial_data_ptr,
    struct rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr)
{
    this->gimbal_current_msg.bullet_speed = serial_data_ptr->bullet_speed;
    this->gimbal_current_msg.pitch_angle = serial_data_ptr->pitch_angle;
    this->gimbal_current_msg.yaw_angle = serial_data_ptr->yaw_angle;

    // 相机内参初始化
    // 只有第一次更新才执行
    if (!this->gimbal.camera_info.init_flag){
        this->gimbal.camera_info.gimbal_name = serial_data_ptr->gimbal;

        if (gimbal.camera_info.camera_matrix.empty() || gimbal.camera_info.dist_coeffs.empty()) {
            if (gimbal.camera_info.gimbal_name == "blackHead") {
                auto camera_matrix_data = this->get_parameter("camera_matrix.data").as_double_array();
                auto dist_coeffs_data = this->get_parameter("dist_coeffs.data").as_double_array();
                
                gimbal.camera_info.camera_matrix = (Mat_<double>(3, 3) <<
                    camera_matrix_data[0], camera_matrix_data[1], camera_matrix_data[2], // fx, cx
                    camera_matrix_data[3], camera_matrix_data[4], camera_matrix_data[5], // fy, cy
                    camera_matrix_data[6], camera_matrix_data[7], camera_matrix_data[8] // 填充项
                );
;
                gimbal.camera_info.dist_coeffs = (Mat_<double>(1, 5) <<
                    dist_coeffs_data[0], dist_coeffs_data[1], dist_coeffs_data[2], // fx, cx
                    dist_coeffs_data[3], dist_coeffs_data[4]
                );

                this->gimbal.camera_info.init_flag = true;
            }
            else if (gimbal.camera_info.gimbal_name == "whiteHead") {
                gimbal.camera_info.camera_matrix = PNP::white_camera_matrix;
                gimbal.camera_info.dist_coeffs = PNP::white_dist_coeffs;
            }
        }
    }
}




/**
 * @brief 重置巡逻模式参数到初始状态
 * @param input_image 输入参考图像（用于获取当前画面尺寸）
 * 
 * 重置内容包括：
 * - 图像尺寸参数
 * - ROI检测区域
 * - 目标轨迹数据
 * - 运动预测参数
 * - 状态控制标志
 */
void ArmorIdentifier::_initialize_patrol(const double& image_height, const double& image_width) {
    this->image_height = image_height;
    this->image_width = image_width;

    this->last_center_x = this->image_width * 0.5;
    this->last_center_y = this->image_height * 0.5;

    this->detector_start_x = this->detector_start_y = 

    this->armorIdx = this->armorAcc = 

    this->detection_count_ = 0;
    this->flight_time = 0.125;
    this->aim_target_points.clear();
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rcia::vision_identify::ArmorIdentifier)
