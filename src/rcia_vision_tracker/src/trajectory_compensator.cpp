#include "trajectory_compensator.hpp"


double Trajectory_compensator::calculate_compensation_angle(double distance, double firing_angle_rad)
{
    if (distance <= 0 || initial_velocity <= 0) {
        throw std::invalid_argument("Invalid distance or velocity");
    }
    
    for (size_t i = 0; i < keys.size(); ++i) {
        ballistic_coefficients[keys[i]] = values[i];
    }

    auto it = ballistic_coefficients.lower_bound(static_cast<int>(distance));
    if (it == ballistic_coefficients.end()) { // 如果超出已知范围，抛出异常或采取其他措施
        throw std::out_of_range("Distance out of range for known ballistic coefficients");
    }
    const double adjusted_bc = it->second * distance;
    const double cos_theta = std::cos(firing_angle_rad);
    
    // 使用 atan2 提高数值稳定性
    const double numerator = adjusted_bc * bullet_mass * acceleration_of_gravity * distance;
    const double denominator = 2 * initial_velocity * initial_velocity * cos_theta * cos_theta;
    return std::atan2(numerator, denominator);  // 返回弧度值
}

void Trajectory_compensator::angle_compensator(const Eigen::Vector3d& target_position, double& pitch_rad, double& current_compen_height, double& current_distance) {
    const double distance =
        std::sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));

    
    const double angle_rad = std::atan2(target_position(2), distance);
    const double compen_rad = calculate_compensation_angle(distance, angle_rad);
    pitch_rad = angle_rad + compen_rad;

    current_compen_height = compen_rad * 180 / M_PI;
    current_distance = distance;

    // 规范化输出
    // std::cout << "angle_rad: " << angle_rad * 180 / M_PI << "°, "
    //           << "compensated: " << compen_rad * 180 / M_PI << "°\n";
}


// Eigen::Vector3d BallisticSystem::transformCoordinates(const Eigen::Vector3d& cameraPoint) const {
//     return cameraPoint - camera_offset;
// }

// // 计算空气阻力加速度
// template <typename T>
// T BallisticSystem::dynamics(T v) const {
//     T dragForce = T(0.5) * T(rho) * T(Cd) * T(A) * v * v;
//     return dragForce / T(mass);
// }

// // 完整弹道仿真流程
// pair<double, vector<array<double, 4>>> BallisticSystem::simulate(double theta_deg, const Eigen::Vector3d& targetCam, bool verbose, const double& delta_pitch) {
//     // 坐标转换
//     Eigen::Vector3d targetGun = transformCoordinates(targetCam);

//     // cout << "targetGun" << targetGun << endl;
//     double horizontalDist = hypot(targetGun[2], targetGun[0]);
//     double heightDiff = abs(targetGun[1]);
//     // cout << "horizontalDist" << horizontalDist << endl;
//     // cout << "heightDiff" << heightDiff << endl;
    
//     // 弹道模拟
//     double theta = theta_deg * 1.0 * M_PI / 180.0;
//     // (delta_pitch / 100.0) + 

//     double vx = v0 * cos(theta);
//     double vy = v0 * sin(theta);
//     double x = 0.0, y = 0.0, t = 0.0;

//     vector<array<double, 4>> trajectory;

//      while (t < max_flight_time && y >= -0.05) {

//         // 记录轨迹点
//         double speed = hypot(vx, vy);
//         trajectory.push_back({t, x, y, speed});

//         // 计算空气阻力
//         double ax, ay;
//         if (speed > 1e-3) {
//             double dragAcc = dynamics(speed);
//             ax = -dragAcc * vx / speed;
//             ay = -g - dragAcc * vy / speed;
//         } else {
//             ax = 0.0;
//             ay = -g;
//         }

//         // 使用 RK4 更新状态
//         rk4_solver(x, y, vx, vy, ax, ay, dt);
//         t += dt;
//     }

//     // 计算最终误差
//     Eigen::Vector2d finalPos(x, y);
//     Eigen::Vector2d targetPos(horizontalDist, 0.0); // heightDiff
//     // cout << "finalPos" << finalPos << endl;

//     double error = (targetPos - finalPos ).norm();
//     // double error = x - horizontalDist;

//     // cout << "theta_deg" << theta_deg << "\ttrajectory error" << error  << endl;

//     return {error, trajectory};
// }

// // 四阶龙格-库塔法（RK4）
// template <typename T>
// void BallisticSystem::rk4_solver(T& x, T& y, T& vx, T& vy, T ax, T ay, T dt) {
//     T k1_vx = ax * dt, k1_vy = ay * dt, k1_x = vx * dt, k1_y = vy * dt;
//     T k2_vx = (ax + T(0.5) * k1_vx) * dt, k2_vy = (ay + T(0.5) * k1_vy) * dt, k2_x = (vx + T(0.5) * k1_vx) * dt, k2_y = (vy + T(0.5) * k1_vy) * dt;
//     T k3_vx = (ax + T(0.5) * k2_vx) * dt, k3_vy = (ay + T(0.5) * k2_vy) * dt, k3_x = (vx + T(0.5) * k2_vx) * dt, k3_y = (vy + T(0.5) * k2_vy) * dt;
//     T k4_vx = (ax + k3_vx) * dt, k4_vy = (ay + k3_vy) * dt, k4_x = (vx + k3_vx) * dt, k4_y = (vy + k3_vy) * dt;

//     vx += (k1_vx + T(2) * k2_vx + T(2) * k3_vx + k4_vx) / T(6);
//     vy += (k1_vy + T(2) * k2_vy + T(2) * k3_vy + k4_vy) / T(6);
//     x += (k1_x + T(2) * k2_x + T(2) * k3_x + k4_x) / T(6);
//     y += (k1_y + T(2) * k2_y + T(2) * k3_y + k4_y) / T(6);
// }

// // 定义 Calibration 误差函数
// struct CalibrationCostFunction {
//     BallisticSystem* system;
//     const vector<pair<double, double>>& realData;

//     CalibrationCostFunction(BallisticSystem* system, const vector<pair<double, double>>& realData)
//         : system(system), realData(realData) {}

//     template <typename T>
//     bool operator()(const T* const Cd, T* residual) const {
//         // 初始化残差
//         residual[0] = T(0.0);

//         // 遍历所有数据点
//         for (const auto& data : realData) {
//             // 将角度转换为弧度
//             T theta_rad = T(data.first * M_PI / 180.0);

//             // 初始条件
//             T vx = T(system->v0) * ceres::cos(theta_rad);
//             T vy = T(system->v0) * ceres::sin(theta_rad);
//             T x = T(0.0), y = T(0.0), t = T(0.0);
//             T dt = T(system->dt); 

//             // 数值积分模拟弹道
//             while (t < T(system->max_flight_time) && y >= T(-0.05)) {
//                 if (x > T(data.second) + T(0.25)) break;

//                 T speed = ceres::sqrt(ceres::pow(vx, 2) + ceres::pow(vy, 2));

//                 T drag_force_x = -T(0.5) * T(system->rho) * (*Cd) * T(system->A) * speed * vx;
//                 T drag_force_y = -T(0.5) * T(system->rho) * (*Cd) * T(system->A) * speed * vy;

//                 // 计算加速度
//                 T ax = drag_force_x / T(system->mass);
//                 T ay = (-T(system->mass) * T(system->g) + drag_force_y) / T(system->mass);

//                 // 使用 RK4 更新状态
//                 system->rk4_solver(x, y, vx, vy, ax, ay, dt);
//                 t += T(system->dt);
//             }

//             // // 计算最终残差
//             Eigen::Matrix<T, 2, 1> finalPos(x, y);
//             Eigen::Matrix<T, 2, 1> targetPos(T(data.second), T(0.0));
//             T error = (targetPos - finalPos).norm();


//             // cout << "误差：" << error << endl;

//             residual[0] += error * error; 

//         }

//         // cout << "Cd" << Cd[0] << endl;
//         // cout << "residual[0] " << residual[0] << endl;

//         return true;
//     }
// };



// // 根据实测数据校准阻力系数
// void BallisticSystem::calibrateCd(const vector<pair<double, double>>& realData) {
//     // 使用 Ceres Solver 进行优化
//     ceres::Problem problem;
//     double initialCd = this->Cd;

//     problem.AddParameterBlock(&initialCd, 1);

//     // 设置阻力系数的边界（典型球形弹丸的范围）
//     problem.SetParameterLowerBound(&initialCd, 0, 0.01);  // min bound
//     problem.SetParameterUpperBound(&initialCd, 0, 1.2);  // max bound
    
//     // for (auto& data : realData) {
//     //     ceres::CostFunction* cost_function =
//     //         new ceres::AutoDiffCostFunction<CalibrationCostFunction, 1, 1>(
//     //             new CalibrationCostFunction(this, data));
//     //     problem.AddResidualBlock(cost_function, nullptr, &initialCd);
//     // }
//     // 创建代价函数
//     ceres::CostFunction* cost_function =
//         new ceres::AutoDiffCostFunction<CalibrationCostFunction, 1, 1>(
//             new CalibrationCostFunction(this, realData));
//     problem.AddResidualBlock(cost_function, nullptr, &initialCd);
    

//     // 配置求解器
//     ceres::Solver::Options options;
//     options.max_num_iterations = 100;
//     // options.linear_solver_type = ceres::DENSE_QR;
//     options.minimizer_type = ceres::TRUST_REGION;
//     options.trust_region_strategy_type = ceres::DOGLEG;
//     // options.minimizer_progress_to_stdout = true;
//     options.gradient_tolerance = 1e-8;
//     options.function_tolerance = 1e-8;

//     // 运行优化
//     ceres::Solver::Summary summary;
//     ceres::Solve(options, &problem, &summary);

//     // 输出优化结果
//     // cout << "优化结果：" << endl;
//     // cout << "是否成功: " << (summary.IsSolutionUsable() ? "是" : "否") << endl;
//     // cout << "初始代价: " << summary.initial_cost << endl;
//     // cout << "最终代价: " << summary.final_cost << endl;
//     // cout << "迭代次数: " << summary.iterations.size() << endl;
//     // cout << "优化时间: " << summary.total_time_in_seconds << " 秒" << endl;
//     // cout << "终止原因: " << summary.message << endl;
//     // cout << summary.BriefReport() << endl; 
    
//     // 更新阻力系数
//     this->Cd = initialCd;
//     cout << "校准后的阻力系数 Cd = " << this->Cd << endl;
// }


// // 定义 AutoAim 误差函数 类
// struct AutoAimCostFunction {
//     BallisticSystem* system;          // 弹道系统对象
//     const Eigen::Vector3d& targetCam; // 目标在相机坐标系中的位置
//     double delta_pitch;

//     AutoAimCostFunction(BallisticSystem* system, const Eigen::Vector3d& targetCam, const double& delta_pitch)
//         : system(system), targetCam(targetCam), delta_pitch(delta_pitch) {}

//     // 实现 operator() 方法
//     template <typename T>
//     bool operator()(const T* const theta, T* residual) const {
//         // 将角度转换为弧度
//         T theta_rad = (theta[0] * T(1.0)) * T(M_PI / 180.0);

//         // 坐标转换
//         Eigen::Vector3d targetGun = system->transformCoordinates(targetCam);

//         // 计算水平距离和高度差
//         T horizontalDist = ceres::sqrt(ceres::pow(T(targetGun[2]), 2) + ceres::pow(T(targetGun[0]), 2));
//         T heightDiff = T(abs(targetGun[1]));

//         // 初始条件
//         T vx = T(system->v0) * ceres::cos(theta_rad);
//         T vy = T(system->v0) * ceres::sin(theta_rad);
//         T x = T(0.0), y = T(0.0), t = T(0.0);
//         T dt = T(system->dt); 

//         while (t < T(system->max_flight_time) && y >= T(0.0)) {
//             if (x > horizontalDist + T(0.0)) break;

//             // 记录轨迹点
//             T speed = ceres::sqrt(ceres::pow(vx, 2) + ceres::pow(vy, 2));

//             // 计算空气阻力
//             T ax, ay;
//             if (speed > T(1e-3)) {
//                 T dragAcc = T(system->dynamics(speed));
//                 ax = -dragAcc * vx / speed;
//                 ay = -T(system->g) - dragAcc * vy / speed;
//             } else {
//                 ax = T(0.0);
//                 ay = -T(system->g);
//             }

//             // 使用 RK4 更新状态
//             system->rk4_solver(x, y, vx, vy, ax, ay, dt);
//             t += T(system->dt);
//         }

//         // // 计算最终残差
//         Eigen::Matrix<T, 2, 1> finalPos(x, y);
//         Eigen::Matrix<T, 2, 1> targetPos(horizontalDist, T(0.0));
//         T error = (targetPos - finalPos).norm();
//         // T error = x - horizontalDist;

//         // cout << "x\t" << x << "\t";
//         // cout << "horizontalDist" << horizontalDist << "\n";
//         // cout << "y\t" << y << "\t";
//         // cout << "heightDiff" << heightDiff << "\n";
//         // cout << "theta_rad" << radiansToDegrees(theta_rad) << "\t";

//         // cout << "误差：" << error << endl;

//         // 将误差作为残差返回
//         residual[0] = error * error;

//         return true;
//     }
// };

// // 自动解算最佳仰角
// pair<double, double> BallisticSystem::autoAim(const Eigen::Vector3d& targetCam, const string& method, const double& delta_pitch, const double& compensation_height) {
//     Eigen::Vector3d targetGun = transformCoordinates(targetCam);

//     auto errorFunc = [&](double theta) -> double {
//         return simulate(theta, targetCam, delta_pitch).first;
//     };

//     // double tol = 0.002;
//     // int maxIter = 100;

//     // // 创建一个包含100个元素的向量，用于存储角度值
//     // vector<double> angles(100);

//     // // 使用 iota 函数填充向量，从 -1.0 开始，依次递增
//     // iota(angles.begin(), angles.end(), -0.0);

//     // // 使用 transform 函数将向量中的每个元素除以系数，将角度值转换为更小的范围
//     // transform(angles.begin(), angles.end(), angles.begin(), [](double x) { return x / 1000.0; });

//     // // 创建一个与 angles 大小相同的向量，用于存储每个角度对应的误差值
//     // vector<double> errors(angles.size());

//     // // 使用 transform 函数计算每个角度对应的误差值，并填充到 errors 向量中
//     // transform(angles.begin(), angles.end(), errors.begin(), errorFunc);

//     // // 找到 errors 向量中最小误差值的索引
//     // int bestIdx = distance(errors.begin(), min_element(errors.begin(), errors.end()));

//     // 获取最佳初始估计角度值
//     // double theta = 0.0; // angles[bestIdx]
//     // cout << "compensation_height " <<  compensation_height << endl;
//     double initialtheta = compensation_height;// compensation_height; //compensation_height;
//     double errorGap = 1.0;
//     double theta = initialtheta;

//     // 输出最佳初始估计角度值
//     // cout << "trajectory initial angle" << initialtheta << endl;

//     if (method == "hybrid") {
//         // double errorGap = errors[bestIdx];
//         // cout  << "trajectory errorGap" << errorGap << endl;
//             // 使用 Ceres Solver 进行 L-BFGS-B 优化

//         ceres::Problem problem;
//         // 创建代价函数
//         ceres::CostFunction* cost_function =
//             new ceres::AutoDiffCostFunction<AutoAimCostFunction, 1, 1>(
//                 new AutoAimCostFunction(this, targetCam, delta_pitch));
//         problem.AddResidualBlock(cost_function, nullptr, &theta);

//         double lowerBound = max(-2.0, theta - errorGap);
//         double upperBound = min(2.5, theta + errorGap);

//         // // // 设置边界
//         problem.SetParameterLowerBound(&theta, 0, lowerBound);
//         problem.SetParameterUpperBound(&theta, 0, upperBound);

//         // 配置求解器
//         // ceres::Solver::Options options;
//         // // options.max_num_iterations = 20;
//         // options.minimizer_type = ceres::LINE_SEARCH;
//         // options.line_search_direction_type = ceres::LBFGS;
//         // options.line_search_type = ceres::WOLFE;
//         // options.linear_solver_type = ceres::DENSE_QR;
//         // // options.line_search_interpolation_type = ceres::CUBIC;
//         // options.trust_region_strategy_type = ceres::DOGLEG;
//         // // options.minimizer_progress_to_stdout = true;


//         ceres::Solver::Options options;
//         options.minimizer_type = ceres::TRUST_REGION; // 使用 TRUST_REGION 优化器
//         options.linear_solver_type = ceres::DENSE_QR; // 或者使用其他线性求解器
//         options.trust_region_strategy_type = ceres::DOGLEG; // 或者使用 LEVENBERG_MARQUARDT
//         options.gradient_tolerance = 1e-8;
//         options.function_tolerance = 1e-8;

//         // 运行优化
//         ceres::Solver::Summary summary;
//         ceres::Solve(options, &problem, &summary);

//         // // 输出优化结果
//         // cout << "优化结果：" << endl;
//         // cout << "是否成功: " << (summary.IsSolutionUsable() ? "是" : "否") << endl;
//         // cout << "初始代价: " << summary.initial_cost << endl;
//         // cout << "最终代价: " << summary.final_cost << endl;
//         // cout << "迭代次数: " << summary.iterations.size() << endl;
//         // cout << "优化时间: " << summary.total_time_in_seconds << " 秒" << endl;
//         // cout << "终止原因: " << summary.message << endl;

//         // cout << "optimized theta" << theta << endl;
//         // 返回优化结果
//         return {theta, errorFunc(theta)};
//     }

//     return {theta, errorFunc(theta)};
// }


// // 绘制轨迹
// void drawTrajectory(cv::Mat& img, const vector<cv::Point2f>& imgpts, cv::Point2f center, float radius) {
//     // for (size_t j = 1; j < imgpts.size(); ++j) {
//     //     cv::line(img, imgpts[j - 1], imgpts[j], cv::Scalar(0, 255, 0), 2);
//     // }

//     cv::circle(img, center, static_cast<int>(radius / 2), cv::Scalar(0, 255, 0), 3);
// }


// // 处理轨迹数据并绘制
// // 此处tvec 和 targetGun重复
// void BallisticSystem::visualize_trajectory(const vector<array<double, 4>>& trajectory, cv::Mat& img, const Mat& rvec, const Mat& tvec, const Eigen::Vector3d& targetCam, const double& optimal_angle, double delta_pitch) {
//     publishTrajectoryToPython(trajectory, targetCam);
    
//     Eigen::Vector3d targetGun = transformCoordinates(targetCam);

//     double deltaYaw = calculate_deltaYaw<double>(tvec);
    
//     // 创建一个新的 vector 来存储修改后的轨迹
//     std::vector<std::array<double, 4>> newTrajectory;

//     // 找到与 tvec.at<double>(2) 差值最小的 data[1]
//     auto minIt = std::min_element(trajectory.begin(), trajectory.end(), [&](const auto& a, const auto& b) {
//         return std::abs(a[1] * 100.0 - tvec.at<double>(2)) < std::abs(b[1] * 100.0 - tvec.at<double>(2));
//     });

//     // 如果找到最小值，则将最小值之前的所有值复制到 newTrajectory
//     // if (minIt != trajectory.end()) {
//     //     newTrajectory.assign(trajectory.begin(), minIt);
//     // } else {
//     //     // 如果未找到最小值，则复制整个 trajectory
//     //     newTrajectory = trajectory;
//     // }
    
//     newTrajectory = trajectory;
//     double sqrtTrajectoryLen = sqrt(newTrajectory.size());

//     // cout << "newTrajectory.size() " << newTrajectory.size() << endl;
    
//     // cout << "delta_pitch " << delta_pitch << endl;

//     for (size_t i = 1; i < newTrajectory.size(); i += 1) {
//         const auto& data = newTrajectory[i];
//         double t = data[0], z = data[1], y = data[2], speed = data[3];

//         // if (z * 100.0 > tvec.at<double>(2)) {
//         //     break;
//         // }

//         // 理论抛物线（无空气阻力）
//         double theory_x = this->v0 * cos(degreesToRadians(optimal_angle)) * t;
//         double theory_y = this->v0 * sin(degreesToRadians(optimal_angle)) * t;

//         // 算出与理论抛物线差值
//         double y_difference = abs(theory_y + y);

//         // double deltaPitch = atan2(tvec.at<double>(1) + (2 * y_difference + y) * -100.0, tvec.at<double>(2));
//         // y = (sqrt(i) * deltaPitch *  tvec.at<double>(2)) / sqrtTrajectoryLen;

//         double x = (sqrt(i) * deltaYaw * z * 100.0) / sqrtTrajectoryLen;

//         double deltaPitch = (atan2((y) * -100.0, tvec.at<double>(2)));
//         // tvec.at<double>(1) + 
//         y = (sqrt(i) * deltaPitch *  tvec.at<double>(2)) / sqrtTrajectoryLen;

//         Mat bullet_rvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);// rvec;
//         Mat bullet_tvec = (cv::Mat_<double>(3, 1) << x, y , z * 100.0);
        
//         // 投影到图像平面
//         vector<cv::Point2f> imgpts;
//         cv::projectPoints(this->axisBullet, bullet_rvec, bullet_tvec, PNP::black_camera_matrix, PNP::black_dist_coeffs, imgpts);

//         // 计算中心点和半径
//         if (!imgpts.empty()) {
//             cv::Moments mu = cv::moments(imgpts);
//             cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
//             float radius = cv::arcLength(imgpts, true) / 4.0f;
            
//             // 绘制轨迹
//             drawTrajectory(img, imgpts, center, radius);
//         }
        
//         addTrajectoryMarkerMsg(bullet_rvec, bullet_tvec, i);
//     }

//     publishTrajectoryMarkerArray();
// }




// void BallisticSystem::publishTrajectoryToPython(const vector<array<double, 4>>& trajectory, const Eigen::Vector3d& targetCam) {
//     Eigen::Vector3d targetGun = transformCoordinates(targetCam);
//     // 创建消息
//     auto message = rcia_vision_interfaces::msg::Trajectory();

//     // 填充消息
//     for (const auto& point : trajectory) {
//         message.time.push_back(point[0]);
//         message.x.push_back(point[1]);
//         message.y.push_back(point[2]);
//         message.z.push_back(point[3]);
//         message.speed.push_back(hypot(point[1], point[2], point[3]));  // 计算速度

//     }

//     message.target_gun_x = targetGun[0];
//     message.target_gun_y = targetGun[1];
//     message.target_gun_z = targetGun[2];

//     // 发布消息
//     trajectory_publisher_->publish(message);
//     // RCLCPP_INFO(this->get_logger(), "Trajectory published!");
// }

// void BallisticSystem::addTrajectoryMarkerMsg(const Mat& publishRvec, const Mat& publishTvec, const int& publishIndex) {
//     // 初始化 Marker
//     visualization_msgs::msg::Marker marker;
//     marker.id = publishIndex;
//     marker.header.frame_id = "odom"; // 父参考系
//     marker.header.stamp = node_->now();
//     marker.action = visualization_msgs::msg::Marker::ADD;

//     // 设置 Marker 类型为球体
//     marker.type = visualization_msgs::msg::Marker::SPHERE;
//     marker.ns = "trajectory";

//     // 设置 Marker 的尺寸
//     marker.scale.x = 0.05; // 球体直径
//     marker.scale.y = 0.05;
//     marker.scale.z = 0.05;

//     // 设置 Marker 的颜色
//     marker.color.a = 0.75; // 不透明度
//     marker.color.r = 0.0; 
//     marker.color.g = 0.95;
//     marker.color.b = 0.25;

//     // 设置 Marker 的位置
//     marker.pose.position.x = publishTvec.at<double>(2) / 100.0; // 转换为米
//     marker.pose.position.y = -publishTvec.at<double>(0) / 100.0;
//     marker.pose.position.z = -publishTvec.at<double>(1) / 100.0;

//     // 设置 Marker 的方向
//     tf2::Quaternion q;
//     q.setRPY(0, 0, 0);
//     marker.pose.orientation.x = q.x();
//     marker.pose.orientation.y = q.y();
//     marker.pose.orientation.z = q.z();
//     marker.pose.orientation.w = q.w();

//     // 将 Marker 添加到数组中
//     trajectoryMarkerArray.markers.emplace_back(marker);

//     // 日志记录
//     // RCLCPP_INFO(node_->get_logger(), "Published trajectory marker with ID: %d", index);
// }


// void BallisticSystem::publishTrajectoryMarkerArray() {

//     // 发布标记数组
//     trajectoryMarker_publish_->publish(trajectoryMarkerArray);

//     // 清空之前的标记数组
//     trajectoryMarkerArray.markers.clear();
// }