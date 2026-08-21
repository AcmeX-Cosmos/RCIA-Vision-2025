
#include "ba_solver.hpp"

#include <eigen3/Eigen/Dense>

#include <cmath>

// 全局变量可做优化
Mat g_reproject_rvec;
Mat g_reproject_tvec;
vector<double> g_camera_intrinsics(4);
vector<double> g_black_dist_coeffs(5);

BA_CLASS::BA_CLASS(std::weak_ptr<rclcpp::Node> n) : node_(n){
    auto node = node_.lock();

    if (node) {
        node->declare_parameter("Ba_Param.trans_vector_optimization", 2.5);
        node->declare_parameter("Ba_Param.yaw_optimization", 0.05);
        node->declare_parameter("Ba_Param.maximum_yaw", 45.0);
        node->declare_parameter("Ba_Param.minimum_yaw", -45.0);
        node->declare_parameter("camera_matrix.data", 
        std::vector<double>{1557.112832049196, 0, 640.0984500155263, 
                                0, 1735.09586187522, 512.0025696784391,
                                0.0, 0.0, 1.0});
        node->declare_parameter("dist_coeffs.data", 
        std::vector<double>{0.02379281673136642,  -2.931918199663403, -0.0001751467257614772, 0.001130183050415235,  42.30457985450274});

        armor_bapose_pub_ = node->create_publisher<rcia_vision_interfaces::msg::ArmorBaposeInfo>("armor_bapose_info", 2);

        armor_marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("armor_marker", 2);
    }

    node.reset();

    init_camra_info();

    // +----------------------------------------------------------------------------------------------------------+
	// |									Option 1: Description of option 1									  |
	// +----------------------------------------------------------------------------------------------------------+
	options.function_tolerance = 1e-6;
	
	options.parameter_tolerance = 1e-6;

	options.minimizer_progress_to_stdout = false;											// ?????????????????,??????????????
		
	options.logging_type = ceres::SILENT;//ceres::PER_MINIMIZER_ITERATION;//ceres::SILENT;	// ??????????????,???????????
		
	options.num_threads = 1;																// ?????????? 1,????????????????
	
	options.preconditioner_type = ceres::JACOBI;											// ?? Jacobi ?????,????????????
	
	// options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

	// options.linear_solver_type = ceres::SPARSE_SCHUR;									// ???? Schur ??????,????????????????
	
	options.linear_solver_type = ceres::DENSE_QR;											// DENSE_QR????????
	
	options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;						// ceres::EIGEN_SPARSE: ?? Eigen ??????????????

	// options.max_num_iterations = 25;

	options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

	// +----------------------------------------------------------------------------------------------------------+
	// |								Option 1: Description of option 1 stop									  |
	// +----------------------------------------------------------------------------------------------------------+


	// +----------------------------------------------------------------------------------------------------------+
	// |								Option 2: Description of option 2 start									  |
	// +----------------------------------------------------------------------------------------------------------+
	
	// options2.function_tolerance = 1e-6;
	
	// options2.parameter_tolerance = 1e-6;

	options2.minimizer_progress_to_stdout = false;											// ?????????????????,??????????????
		
	options2.logging_type = ceres::SILENT;//ceres::PER_MINIMIZER_ITERATION;//ceres::SILENT;	// ??????????????,???????????
		
	options2.num_threads = 1;																// ?????????? 1,????????????????
	
	options2.preconditioner_type = ceres::JACOBI;											// ?? Jacobi ?????,????????????
	
	// options2.linear_solver_type = ceres::SPARSE_SCHUR;									// ???? Schur ??????,????????????????
	
	options2.linear_solver_type = ceres::DENSE_QR;											// DENSE_QR????????
	
	options2.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;						// ceres::EIGEN_SPARSE: ?? Eigen ??????????????

	options2.max_num_iterations = 5;

	options2.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

	// +----------------------------------------------------------------------------------------------------------+
	// |								Option 2: Description of option 2 stop									  |
	// +----------------------------------------------------------------------------------------------------------+

}

void BA_CLASS::init_camra_info(){
    auto node = node_.lock();
    auto camera_matrix_data = node->get_parameter("camera_matrix.data").as_double_array();
    g_camera_intrinsics = { 
        camera_matrix_data[0], // 焦距 fx
        camera_matrix_data[4], // 焦距 fy
        camera_matrix_data[2], // 主点 cx
        camera_matrix_data[5]  // 主点 cy
    };
    auto dist_coeffs_data = node->get_parameter("dist_coeffs.data").as_double_array();
    g_black_dist_coeffs ={
        dist_coeffs_data[0],
        dist_coeffs_data[1],
        dist_coeffs_data[2],
        dist_coeffs_data[3],
        dist_coeffs_data[4]
    };
}

/**
 * @class ReprojectionError2
 * @brief 用于计算3D点到2D点重投影误差的类。
 *        该类通常用于优化框架（如Ceres Solver）中，用于计算残差。
 */
class ReprojectionError2
{
public:
    /**
     * @brief 构造函数，初始化观测的2D点和旋转向量。
     * @param observed_ 观测到的2D点集合。
     * @param rvec_ 旋转向量（Rodrigues格式）。
     */
    ReprojectionError2(const vector<Point2f>& observed_, const vector<double>& rvec_)
        : observed(observed_), rvec(rvec_) {}

    /**
     * @brief 重载括号运算符，用于计算残差。
     * @param initial_tvecX 初始平移向量的X分量。
     * @param initial_tvecY 初始平移向量的Y分量。
     * @param initial_tvecZ 初始平移向量的Z分量。
     * @param residuals 输出的残差数组。
     * @return 返回布尔值，表示计算是否成功。
     */
    template<typename T>
    bool operator()(const T* const initial_tvecX, const T* const initial_tvecY, const T* const initial_tvecZ, T* residuals) const
    {   
        // 将旋转向量转换为模板类型数组
        T rvec_array[3];
        rvec_array[0] = T(rvec[0]); 
        rvec_array[1] = T(rvec[1]); 
        rvec_array[2] = T(rvec[2]); 

        double sum_X = 0, sum_Y = 0, sum_Z = 0; // 用于累加3D点的坐标

        for (size_t point_idx = 0; point_idx < observed.size(); ++point_idx) {
            // 获取对应的3D点
            const Point3f& repoints = PNP::object_3d_points_small[point_idx];
            Vector3d rp3d(repoints.x, repoints.y, repoints.z);

            // 定义3D点的初始坐标
            T pt1[3] = {T(rp3d.x()), T(rp3d.y()), T(rp3d.z())};  
            T pt2[3]; // 旋转后的3D点坐标

            // 使用旋转向量对3D点进行旋转，并加上平移向量
            ceres::AngleAxisRotatePoint(rvec_array, pt1, pt2);
            pt2[0] += *initial_tvecX;
            pt2[1] += *initial_tvecY;
            pt2[2] += *initial_tvecZ;

            // 根据模板类型累加3D点的坐标
            if constexpr (std::is_same<T, ceres::Jet<double, 3>>::value) {
                sum_X += pt2[0].a;
                sum_Y += pt2[1].a;
                sum_Z += pt2[2].a;
            } else {
                sum_X += pt2[0];
                sum_Y += pt2[1];
                sum_Z += pt2[2];
            }

            // 获取相机内参
            const T fx = T(g_camera_intrinsics[0]);
            const T fy = T(g_camera_intrinsics[1]);
            const T cx = T(g_camera_intrinsics[2]);
            const T cy = T(g_camera_intrinsics[3]);

            // 计算重投影后的2D点坐标
            const T xp = T(fx * (pt2[0] / pt2[2]) + cx);
            const T yp = T(fy * (pt2[1] / pt2[2]) + cy);

            // 获取观测的2D点
            Point2f points = observed[point_idx];
            Eigen::Vector2d p2d(points.x, points.y);
            const T u = T(p2d.x());
            const T v = T(p2d.y());

            // 计算残差
            residuals[2 * point_idx] = xp - u;
            residuals[2 * point_idx + 1] = yp - v;

            // 打印调试信息（可选）
            // cout << "xp: " << xp << " u: " << u << endl;
            // cout << "yp: " << yp << " v: " << v << endl;
        }

        // 计算平均3D点坐标
        double avg_X = sum_X / 4.0;
        double avg_Y = sum_Y / 4.0;
        double avg_Z = sum_Z / 4.0;
        g_reproject_tvec = (cv::Mat_<double>(3, 1) << avg_X, avg_Y, avg_Z);

        // 打印残差信息（可选）
        // cout << "\n# -------------------------- # residuals Start # ------------------------ # " << endl;
        // cout << "residuals[0]" << residuals[0] << "\t\tresiduals[1]" << residuals[1] << endl;
        // cout << "residuals[2]" << residuals[2] << "\t\tresiduals[3]" << residuals[3] << endl;
        // cout << "residuals[4]" << residuals[4] << "\t\tresiduals[5]" << residuals[5] << endl;
        // cout << "residuals[6]" << residuals[6] << "\t\tresiduals[7]" << residuals[7] << endl;
        // cout << "# -------------------------- # residuals Stop # ------------------------ # \n" << endl;

        return true;
    }  

private:
    vector<Point2f> observed; // 观测到的2D点集合
    vector<double> rvec;      // 旋转向量
};



/**
 * @brief 将旋转矩阵转换为旋转向量（轴角表示）。
 *        该函数支持模板类型，兼容普通浮点数和 ceres::Jet 类型。
 * 		  此处应用于 ReprojectionError1 优化
 * @param R 输入的3x3旋转矩阵，使用二维向量表示。
 * @return 返回旋转向量（轴角表示），以一维向量形式存储。
 */
template<typename T>
std::vector<T> rotationMatrixToAxisAngle(const std::vector<std::vector<T>>& R) {
    // 创建一个3x3的双精度矩阵，用于存储旋转矩阵
    cv::Mat_<double> R_double(3, 3);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            // 根据模板类型将旋转矩阵元素转换为 double 类型
            if constexpr (std::is_same<T, ceres::Jet<double, 1>>::value) {
                R_double(i, j) = static_cast<double>(R[i][j].a); // 提取 Jet 类型的值
            } else {
                R_double(i, j) = static_cast<double>(R[i][j]); // 直接转换为 double
            }
        }
    }

    // 定义一个3x1的旋转向量（轴角表示）
    cv::Mat_<double> rvec(3, 1);

    // 使用 OpenCV 的 Rodrigues 函数将旋转矩阵转换为旋转向量
    cv::Rodrigues(R_double, rvec);

    // 将旋转向量转换为模板类型并返回
    std::vector<T> rvec_jet;
    for (int i = 0; i < 3; ++i) {
        rvec_jet.push_back(T(rvec(i, 0))); // 转换为模板类型并存入结果向量
    }

    return rvec_jet;
}

template<typename T>
std::vector<std::vector<T>> multiplyMatrices(const std::vector<std::vector<T>>& A, const std::vector<std::vector<T>>& B) {
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matA(A.size(), A[0].size());
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matB(B.size(), B[0].size());

    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < A[0].size(); ++j) {
            matA(i, j) = A[i][j];
        }
    }

    for (size_t i = 0; i < B.size(); ++i) {
        for (size_t j = 0; j < B[0].size(); ++j) {
            matB(i, j) = B[i][j];
        }
    }

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> result = matA * matB;

    std::vector<std::vector<T>> res(result.rows(), std::vector<T>(result.cols()));
    for (size_t i = 0; i < result.rows(); ++i) {
        for (size_t j = 0; j < result.cols(); ++j) {
            res[i][j] = result(i, j);
        }
    }

    return res;
}


/**
 * @class ReprojectionError1
 * @brief 用于计算3D点到2D点重投影误差的类。
 *        该类通常用于优化框架（如Ceres Solver）中，用于计算残差。
 */
class ReprojectionError1
{
public:
    /**
     * @brief 构造函数，初始化观测的2D点和初始平移向量。
     * @param observed_ 观测到的2D点集合。
     * @param tvec_ 初始平移向量。
     */
    ReprojectionError1(const vector<Point2f>& observed_, const vector<double>& tvec_, const double target_pitch_)
        : observed(observed_), tvec(tvec_), target_pitch(target_pitch_) {}

    /**
     * @brief 重载括号运算符，用于计算残差。
     * @param initial_yaw 输入的初始偏航角（yaw）。 
     * @param residuals 输出的残差数组。
     * @return 返回布尔值，表示计算是否成功。
     */
    template<typename T>
    bool operator()(const T* const initial_yaw, T* residuals) const
    {   
        // 获取输入的偏航角
        const T& yaw = *initial_yaw;
        
        // 定义 pitch 和 roll 的角度（单位：弧度）
        T pitch = T(target_pitch) * M_PI / T(180.0);       // pitch = 15 度
        T roll = T(0.0);                            // roll = 0 度

        // 构造绕 x 轴的旋转矩阵
        std::vector<std::vector<T>> Rx(3, std::vector<T>(3));
        Rx[0][0] = T(1.0); Rx[0][1] = T(0.0); Rx[0][2] = T(0.0);
        Rx[1][0] = T(0.0); Rx[1][1] = cos(pitch); Rx[1][2] = -sin(pitch);
        Rx[2][0] = T(0.0); Rx[2][1] = sin(pitch); Rx[2][2] = cos(pitch);

        // 构造绕 y 轴的旋转矩阵
        std::vector<std::vector<T>> Ry(3, std::vector<T>(3));
        Ry[0][0] = cos(yaw); Ry[0][1] = T(0.0); Ry[0][2] = sin(yaw);
        Ry[1][0] = T(0.0); Ry[1][1] = T(1.0); Ry[1][2] = T(0.0);
        Ry[2][0] = -sin(yaw); Ry[2][1] = T(0.0); Ry[2][2] = cos(yaw);


        // 构造绕 z 轴的旋转矩阵
        std::vector<std::vector<T>> Rz(3, std::vector<T>(3));
        Rz[0][0] = cos(roll); Rz[0][1] = -sin(roll); Rz[0][2] = T(0.0);
        Rz[1][0] = sin(roll); Rz[1][1] = cos(roll); Rz[1][2] = T(0.0);
        Rz[2][0] = T(0.0); Rz[2][1] = T(0.0); Rz[2][2] = T(1.0);

        // 合并旋转矩阵：R = Rz * Ry * Rx
        std::vector<std::vector<T>> P15R0_Rmatrix = multiplyMatrices(multiplyMatrices(Rz, Ry), Rx);

        // 将旋转矩阵转换为旋转向量（轴角表示）
        auto axis_angle = rotationMatrixToAxisAngle(P15R0_Rmatrix);

        // 定义旋转向量
        T P15R0_rvec[3];
        for (size_t i = 0; i < axis_angle.size(); ++i) {
            if constexpr (std::is_same<T, ceres::Jet<double, 1>>::value) {
                P15R0_rvec[i] = axis_angle[i]; 
            } else {
                P15R0_rvec[i] = T(axis_angle[i]); 
            }
        }

        // 打印旋转向量（可选）
        // for (const auto& val : P15R0_rvec) {
        //     if constexpr (std::is_same<T, ceres::Jet<double, 1>>::value) {
        //         std::cout << val.a << " "; // 提取 Jet 类型的值
        //     } else {
        //         std::cout << val << " ";
        //     }
        // }
        // std::cout << std::endl;

        // 遍历所有观测点，计算残差
        for (size_t point_idx = 0; point_idx < observed.size(); ++point_idx) {
            const Point3f& repoints = PNP::object_3d_points_small[point_idx];
            Vector3d rp3d(repoints.x, repoints.y, repoints.z);

            // 定义3D点的初始坐标
            T pt1[3] = {T(rp3d.x()), T(rp3d.y()), T(rp3d.z())};  
            T pt2[3];

            // 使用旋转向量对3D点进行旋转，并加上平移向量
            ceres::AngleAxisRotatePoint(P15R0_rvec, pt1, pt2);
            pt2[0] += tvec[0];
            pt2[1] += tvec[1];
            pt2[2] += tvec[2];

            // 获取相机内参
            const T fx = T(g_camera_intrinsics[0]);
            const T fy = T(g_camera_intrinsics[1]);
            const T cx = T(g_camera_intrinsics[2]);
            const T cy = T(g_camera_intrinsics[3]);

            // 计算重投影后的2D点坐标
            const T xp = T(fx * (pt2[0] / pt2[2]) + cx);
            const T yp = T(fy * (pt2[1] / pt2[2]) + cy);

            // 获取观测的2D点
            Point2f points = observed[point_idx];
            Eigen::Vector2d p2d(points.x, points.y);
            const T u = T(p2d.x());
            const T v = T(p2d.y());

            // 计算残差
            residuals[2 * point_idx] = xp - u;
            residuals[2 * point_idx + 1] = yp - v;

            // // 打印调试信息（可选）
            // cout << "xp: " << xp << " u: " << u << endl;
            // cout << "yp: " << yp << " v: " << v << endl;
        }

        // 打印残差信息（可选）
        // cout << "\n# -------------------------- # residuals Start # ------------------------ # " << endl;
        // cout << "residuals[0]" << residuals[0] << "\t\tresiduals[1]" << residuals[1] << endl;
        // cout << "residuals[2]" << residuals[2] << "\t\tresiduals[3]" << residuals[3] << endl;
        // cout << "residuals[4]" << residuals[4] << "\t\tresiduals[5]" << residuals[5] << endl;
        // cout << "residuals[6]" << residuals[6] << "\t\tresiduals[7]" << residuals[7] << endl;
        // cout << "# -------------------------- # residuals Stop # ------------------------ # \n" << endl;

        // 根据模板类型存储旋转向量和平移向量
        if constexpr (std::is_same<T, ceres::Jet<double, 1>>::value) {
            g_reproject_rvec = (cv::Mat_<double>(3, 1) << P15R0_rvec[0].a, P15R0_rvec[1].a, P15R0_rvec[2].a);
            // g_reproject_tvec = (cv::Mat_<double>(3, 1) << tvec[0], tvec[1], tvec[2]);
        } else {
            g_reproject_rvec = (cv::Mat_<double>(3, 1) << P15R0_rvec[0], P15R0_rvec[1], P15R0_rvec[2]);
            // g_reproject_tvec = (cv::Mat_<double>(3, 1) << tvec[0], tvec[1], tvec[2]);
        }

        return true;
    }  

private:
    vector<Point2f> observed; // 观测到的2D点集合
    vector<double> tvec;      // 初始平移向量
    double target_pitch;

};


/**
 * @brief 执行束调整（Bundle Adjustment）优化。
 *        该函数使用 Ceres Solver 对外参矩阵进行优化，包括平移向量和偏航角（yaw）。
 * @param extrinsics 输入/输出的外参矩阵集合，包含旋转向量和平移向量。
 * @param image_points 每张图像对应的2D观测点集合。
 */
void BA_CLASS::bundle_adjustment(
    vector<Mat>& extrinsics,
    vector<vector<Point2f>>& image_points,
    const uint8_t& armor_pattern_idx)
{
    auto node = node_.lock();

    if (node)
    {
        ba_param_.trans_vector_optimization = node->get_parameter("Ba_Param.trans_vector_optimization").as_double();
        ba_param_.yaw_optimization = node->get_parameter("Ba_Param.yaw_optimization").as_double();
        ba_param_.minimum_yaw = node->get_parameter("Ba_Param.minimum_yaw").as_double();
        ba_param_.maximum_yaw = node->get_parameter("Ba_Param.maximum_yaw").as_double();
    }

    node.reset();

    // 定义 Cauchy 损失函数，用于鲁棒优化
    ceres::CauchyLoss *loss_function2 = new ceres::CauchyLoss(ba_param_.trans_vector_optimization); // 用于平移向量优化
    ceres::CauchyLoss *loss_function = new ceres::CauchyLoss(ba_param_.yaw_optimization);           // 用于偏航角（yaw）优化
    
    ceres::Problem problem2;

    // 遍历每张图像的外参矩阵和对应的2D点
    for (size_t img_idx = 0; img_idx < extrinsics.size(); ++img_idx)
    {
        // 获取当前图像的2D观测点
        vector<Point2f> p2d = image_points[img_idx];

        // 提取旋转向量（rvec）
        vector<double> rvec;
        rvec.emplace_back(extrinsics[img_idx].at<double>(0, 0));
        rvec.emplace_back(extrinsics[img_idx].at<double>(0, 1));
        rvec.emplace_back(extrinsics[img_idx].at<double>(0, 2));

        // 初始化平移向量的X分量，并设置参数块及上下界
        double initial_tvecX_value = extrinsics[img_idx].at<double>(0, 3);
        double* initial_tvecX = &initial_tvecX_value;
        problem2.AddParameterBlock(initial_tvecX, 1);
        problem2.SetParameterLowerBound(initial_tvecX, 0, extrinsics[img_idx].at<double>(0, 3) - 0.05);
        problem2.SetParameterUpperBound(initial_tvecX, 0, extrinsics[img_idx].at<double>(0, 3) + 0.05);

        // 初始化平移向量的Y分量，并设置参数块及上下界
        double initial_tvecY_value = extrinsics[img_idx].at<double>(0, 4);
        double* initial_tvecY = &initial_tvecY_value;
        problem2.AddParameterBlock(initial_tvecY, 1);
        problem2.SetParameterLowerBound(initial_tvecY, 0, extrinsics[img_idx].at<double>(0, 4) - 0.05);
        problem2.SetParameterUpperBound(initial_tvecY, 0, extrinsics[img_idx].at<double>(0, 4) + 0.05);

        // 初始化平移向量的Z分量，并设置参数块及上下界
        double initial_tvecZ_value = extrinsics[img_idx].at<double>(0, 5);
        double* initial_tvecZ = &initial_tvecZ_value;
        problem2.AddParameterBlock(initial_tvecZ, 1);
        problem2.SetParameterLowerBound(initial_tvecZ, 0, extrinsics[img_idx].at<double>(0, 5) - 0.05);
        problem2.SetParameterUpperBound(initial_tvecZ, 0, extrinsics[img_idx].at<double>(0, 5) + 0.05);

        // 创建重投影误差的成本函数
        ceres::CostFunction* cost_function2 = 
            new ceres::AutoDiffCostFunction<ReprojectionError2, 8, 1, 1, 1>(
                new ReprojectionError2(p2d, rvec));

        // 将成本函数添加到问题中，并关联损失函数和参数块
        problem2.AddResidualBlock(cost_function2,
                                  loss_function2, 
                                  initial_tvecX,
                                  initial_tvecY,
                                  initial_tvecZ);
    }

    // // 执行优化并获取结果
    // ceres::Solver::Summary summary2;
    // ceres::Solve(this->options2, &problem2, &summary2);

    // // 打印优化结果（可选）
    // std::cout << "优化结果：" << std::endl;
    // std::cout << "是否成功: " << (summary2.IsSolutionUsable() ? "是" : "否") << std::endl;
    // std::cout << "初始代价: " << summary2.initial_cost << std::endl;
    // std::cout << "最终代价: " << summary2.final_cost << std::endl;
    // std::cout << "迭代次数: " << summary2.iterations.size() << std::endl;
    // std::cout << "优化时间: " << summary2.total_time_in_seconds << " 秒" << std::endl;
    // std::cout << "终止原因: " << summary2.message << std::endl;
    
    // // 更新外参矩阵中的平移向量
    // extrinsics.back().at<double>(0, 3) = g_reproject_tvec.at<double>(0);
    // extrinsics.back().at<double>(0, 4) = g_reproject_tvec.at<double>(1);
    // extrinsics.back().at<double>(0, 5) = g_reproject_tvec.at<double>(2);

    // 创建新的优化问题，用于优化偏航角（yaw）

    ceres::Problem problem;
    double target_pitch_deg = armor_pattern_idx == 6 ? TARGET_PITCH_DAG : -TARGET_PITCH_DAG;

    for (size_t img_idx = 0; img_idx < extrinsics.size(); ++img_idx)
    {   
        // 获取当前图像的2D观测点
        vector<Point2f> p2d = image_points[img_idx];

        // 提取平移向量（tvec）
        vector<double> tvec = {extrinsics[img_idx].at<double>(0, 3),
                               extrinsics[img_idx].at<double>(0, 4),
                               extrinsics[img_idx].at<double>(0, 5)};

        // 初始化偏航角（yaw），并设置参数块及上下界
        double initial_yaw_value = extrinsics[img_idx].at<double>(0, 1);
        double* initial_yaw = &initial_yaw_value;
        problem.AddParameterBlock(initial_yaw, 1);

        // // 打印优化前的偏航角（转换为角度制）
        // std::cout << "优化前的yaw (idx): " << img_idx << "): " 
        // << initial_yaw_value * (180.0 / M_PI) << " 度" << std::endl;

        double min_yaw = ba_param_.minimum_yaw* M_PI / 180.0;            // 最小偏航角
        double max_yaw = ba_param_.maximum_yaw* M_PI / 180.0;            // 最大偏航角
        problem.SetParameterLowerBound(initial_yaw, 0, min_yaw);
        problem.SetParameterUpperBound(initial_yaw, 0, max_yaw);

        // 创建重投影误差的成本函数
        ceres::CostFunction* cost_function = 
            new ceres::AutoDiffCostFunction<ReprojectionError1, 8, 1>(
                new ReprojectionError1(p2d, tvec, target_pitch_deg));

        // 将成本函数添加到问题中，并关联损失函数和参数块
        problem.AddResidualBlock(cost_function,
                                 loss_function, 
                                 initial_yaw);
    }

    // 执行优化并获取结果
    ceres::Solver::Summary summary;
    ceres::Solve(this->options, &problem, &summary);

    // // 打印优化结果（可选）
    // std::cout << "优化结果：" << std::endl;
    // std::cout << "是否成功: " << (summary.IsSolutionUsable() ? "是" : "否") << std::endl;
    // std::cout << "初始代价: " << summary.initial_cost << std::endl;
    // std::cout << "最终代价: " << summary.final_cost << std::endl;
    // std::cout << "迭代次数: " << summary.iterations.size() << std::endl;
    // std::cout << "优化时间: " << summary.total_time_in_seconds << " 秒" << std::endl;
    // std::cout << "终止原因: " << summary.message << std::endl;

    // 更新外参矩阵中的旋转向量
    extrinsics.back().at<double>(0, 0) = g_reproject_rvec.at<double>(0);
    extrinsics.back().at<double>(0, 1) = g_reproject_rvec.at<double>(1);
    extrinsics.back().at<double>(0, 2) = g_reproject_rvec.at<double>(2);
}


/**
 * @brief 调整优化后的数据，包括旋转向量、平移向量和2D点。
 *        该函数会将新的优化数据加入队列，并根据条件清理旧数据。
 * @param object_2d_point 输入的2D点集合，表示当前帧的观测点。
 */
void BA_CLASS::adjust_optimized_data(vector<Point2f>& object_2d_point) {
    // 将优化后的旋转向量、平移向量和2D点分别加入队列
    rotations.emplace_back(g_reproject_rvec);
    translations.emplace_back(g_reproject_tvec);
    image_points.emplace_back(object_2d_point);

    // 如果队列中的数据超过一定数量（例如3），删除最早的数据以保持队列长度
    if (rotations.size() > this->optimizeLength_) {
        rotations.erase(rotations.begin());
        translations.erase(translations.begin());
        image_points.erase(image_points.begin());
    }

    // 如果队列中有多个数据，计算平移向量之间的距离差异
    if (rotations.size() > 1) {
        double disDifference = sqrt(pow(translations[translations.size() - 2].at<double>(0) - g_reproject_tvec.at<double>(0), 2));
        
        // 如果距离差异过大（例如超过20.0），清空所有队列数据
        if (disDifference > 0.2) {
            rotations.clear();
            translations.clear();
            image_points.clear();
            // cout << "armor jump ~~~" << endl; // 打印调试信息（可选）
        }
    }
}



/**
 * @brief 创建外参矩阵集合。
 *        该函数根据输入的旋转向量和平移向量集合，生成外参矩阵集合。
 * @param rotations 旋转向量集合。
 * @param translations 平移向量集合。
 * @return 返回外参矩阵集合。
 */
vector<Mat> BA_CLASS::createExtrinsicsMatrix(const vector<Mat>& rotations, const vector<Mat>& translations) {
    size_t num_matrices = rotations.size();
    vector<Mat> extrinsics(num_matrices, Mat(6, 1, CV_64FC1));

    for (size_t i = 0; i < num_matrices; ++i) {
        Mat& extrinsic = extrinsics[i];
        Mat r = rotations[i];
        Mat t = translations[i];

        for (int j = 0; j < this->optimizeLength_; ++j) {
            extrinsic.at<double>(j, 0) = r.at<double>(j, 0);
        }
        for (int j = 0; j < this->optimizeLength_; ++j) {
            extrinsic.at<double>(j + 3, 0) = t.at<double>(j, 0);
        }
    }

    return extrinsics;
}

/**
 * @brief 修改姿态信息，根据旋转向量计算装甲板的俯仰角（pitch）、偏航角（yaw）和滚转角（roll）。
 *        该函数将旋转向量转换为旋转矩阵，并从中提取姿态角度。
 */
void BA_CLASS::calculate_armor_eulerAngles() {
    cv::Mat rotM;

    // 使用 Rodrigues 公式将旋转向量转换为旋转矩阵
    Rodrigues(g_reproject_rvec, rotM);
    // cout << "g_reproject_rvec" << g_reproject_rvec << endl; // 打印旋转向量（可选）

    // 计算俯仰角（ba_armor_pitch）
    armor_bapose_msg_.euler.x = atan2(rotM.at<double>(2, 1), rotM.at<double>(2, 2)) * (180.0 / CV_PI);

    if ((rotM.at<double>(2, 1) * rotM.at<double>(2, 1) + rotM.at<double>(2, 2)) < 0) {
        rotM.at<double>(2, 2) = 0;
    }
    // 计算偏航角（ba_armor_yaw）
    armor_bapose_msg_.euler.y = atan2(-rotM.at<double>(2, 0), 
                                          sqrt(rotM.at<double>(2, 1) * rotM.at<double>(2, 1) + rotM.at<double>(2, 2)) * rotM.at<double>(2, 2)) * (180.0 / CV_PI);

    // 计算滚转角（ba_armor_roll）
    armor_bapose_msg_.euler.z = atan2(rotM.at<double>(1, 0), rotM.at<double>(0, 0)) * (180.0 / CV_PI);
}




/**
 * @brief 求解器函数，用于处理目标检测和姿态估计。
 *        该函数根据输入的2D点、旋转向量、平移向量等信息，计算优化后的姿态和射击参数。
 * @param armor_identify_msg 指向装甲板目标信息，包含姿态角度等信息。
 */
void BA_CLASS::Solver(
    rcia_vision_interfaces::msg::ArmorIdentifyInfo::SharedPtr armor_identify_msg)
{   
    if (armor_identify_msg->armor_type == "null") {
        publish_armor_bapose(armor_identify_msg);
    }
    else{
        vector<Point2f> object_2d_point;
            
        for (const auto &p2d : armor_identify_msg->armor_2d_points) {
            // cout << "p2d.x: " << p2d.x << " p2d.y: " << p2d.y << endl;
            object_2d_point.emplace_back(Point2f(p2d.x, p2d.y));
        }

        g_reproject_rvec = (cv::Mat_<double>(3, 1) << armor_identify_msg->pnp_armor_rvec.x, armor_identify_msg->pnp_armor_rvec.y, armor_identify_msg->pnp_armor_rvec.z);
        g_reproject_tvec = (cv::Mat_<double>(3, 1) << armor_identify_msg->pnp_armor_tvec.x, armor_identify_msg->pnp_armor_tvec.y, armor_identify_msg->pnp_armor_tvec.z);

        // 调整优化数据
        adjust_optimized_data(object_2d_point);

        if (rotations.size() > 1) {
            vector<Mat> extrinsics;
            for (size_t i = 0; i < rotations.size(); ++i) {	
                Mat extrinsic(6, 1, CV_64FC1);
                Mat r = rotations[i];
                r.copyTo(extrinsic.rowRange(0, 3));		
                translations[i].copyTo(extrinsic.rowRange(3, 6));
                extrinsics.emplace_back(extrinsic);
            }

            // 此处应该要在惯性系下进行Ba优化才对
            // 执行Ba优化
            bundle_adjustment(extrinsics, image_points, armor_identify_msg->armor_pattern_idx);

            calculate_armor_eulerAngles();

            publish_armor_bapose(armor_identify_msg);

            publish_armor_marker();

        }
    }
}


// +----------------------------------------------------------------------------------------------------------+
// |								ROS2 publish information segment start									  |
// +----------------------------------------------------------------------------------------------------------+
void BA_CLASS::publish_armor_bapose(rcia_vision_interfaces::msg::ArmorIdentifyInfo::SharedPtr armor_identify_msg) {
    auto node_shared = node_.lock();

    if (node_shared) {
        // 此处应该是camera_optical_frame 不过暂时先写camera_link
        armor_bapose_msg_.header.frame_id = "camera_link";

        armor_bapose_msg_.header.stamp = node_shared->now();
        
        if (armor_identify_msg->armor_type == "null") {
            armor_bapose_msg_.armor_type = "null";
        }
        else{
            armor_bapose_msg_.armor_type = armor_identify_msg->armor_type;
            armor_bapose_msg_.armor_pattern_acc = armor_identify_msg->armor_pattern_acc;
            armor_bapose_msg_.armor_pattern_idx = armor_identify_msg->armor_pattern_idx;
            
            // 坐标系转换：OpenCV-> ROS
            // (OpenCV: x-right,y-down,z-forward -> ROS: x-forward,y-left,z-up)
            armor_bapose_msg_.pose.position.x = g_reproject_tvec.at<double>(2); 
            armor_bapose_msg_.pose.position.y = -g_reproject_tvec.at<double>(0);
            armor_bapose_msg_.pose.position.z = -g_reproject_tvec.at<double>(1);
        
            tf2::Quaternion q;
            q.setRPY(0,                                             // roll
                -g_reproject_rvec.at<double>(0),                    // pitch
                -g_reproject_rvec.at<double>(1));                   // yaw
                        
            armor_bapose_msg_.pose.orientation.x = q.x();
            armor_bapose_msg_.pose.orientation.y = q.y();
            armor_bapose_msg_.pose.orientation.z = q.z();
            armor_bapose_msg_.pose.orientation.w = q.w();
        }

        armor_bapose_pub_->publish(armor_bapose_msg_);
    }
}


void BA_CLASS::publish_armor_marker() {
    auto node_shared = node_.lock();
    if (node_shared) {
        // 初始化 armor_marker_
        visualization_msgs::msg::Marker armor_marker_;

        // 此处应该是camera_optical_frame 不过暂时先写camera_link
        armor_marker_.header.frame_id = "camera_link";
        
        armor_marker_.id = 0;

        armor_marker_.header.stamp = node_shared->now();
        armor_marker_.action = visualization_msgs::msg::Marker::ADD;
        armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
        armor_marker_.ns = "armor";

        armor_marker_.scale.x = 0.025;
        armor_marker_.scale.y = 0.175;
        armor_marker_.scale.z = 0.1;

        armor_marker_.color.a = 1.0;
        armor_marker_.color.r = 0.0;
        armor_marker_.color.g = 0.75;
        armor_marker_.color.b = 0.95;
        armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

        // 坐标系转换：OpenCV-> ROS
        // (OpenCV: x-right,y-down,z-forward -> ROS: x-forward,y-left,z-up)
        armor_marker_.pose.position.x = g_reproject_tvec.at<double>(2);
        armor_marker_.pose.position.y = -g_reproject_tvec.at<double>(0);
        armor_marker_.pose.position.z = -g_reproject_tvec.at<double>(1);

        // 设置 armor_marker_ 的方向
        tf2::Quaternion q;
        q.setRPY(0,                             // roll
            -g_reproject_rvec.at<double>(0),    // pitch
            -g_reproject_rvec.at<double>(1));   // yaw
                
        armor_marker_.pose.orientation.x = q.x();
        armor_marker_.pose.orientation.y = q.y();
        armor_marker_.pose.orientation.z = q.z();
        armor_marker_.pose.orientation.w = q.w();

        armor_marker_pub_->publish(armor_marker_);
    }
}

// +----------------------------------------------------------------------------------------------------------+
// |								ROS2 publish information segment stop									  |
// +----------------------------------------------------------------------------------------------------------+