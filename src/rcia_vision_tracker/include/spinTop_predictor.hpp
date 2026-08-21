// spinTop_predictor.hpp
#ifndef SPINTOP_PREDICTOR_HPP
#define SPINTOP_PREDICTOR_HPP

#include "Type.hpp"

class ExtendedKalmanFilter {
public:
    // 构造函数
    explicit ExtendedKalmanFilter();

    MatrixXd exkalman_predict(const double &pdt_);
    MatrixXd exkalman_update(const VectorXd &Z);

    Vector3d getArmorPositionFromState(const Eigen::VectorXd &x) noexcept;

    void setState(const Eigen::VectorXd &x) noexcept;

    // 过程噪声矩阵
    double s2q_x_, s2q_y_, s2q_z_, s2qyaw_, s2qr_;

    // 观测噪声矩阵
    double r_x_, r_y_, r_z_, r_yaw_;

private:
    // 辅助计算函数
    VectorXd f(const Eigen::VectorXd& X_prev, double pdt_);
    VectorXd h(const VectorXd & x);
    MatrixXd jacobian_h(const Eigen::VectorXd & x);
    MatrixXd jacobian_f(double pdt_);

    // 初始化相关
    void initializeR();
    void initializeQ();

    // 状态更新相关
    void update_R(const VectorXd &Z);
    void update_Q(double pdt_);

private:
    // 状态矩阵
    Eigen::MatrixXd P; // 协方差矩阵
    Eigen::MatrixXd Q; // 过程噪声协方差矩阵
    Eigen::MatrixXd H; // 观测矩阵
    Eigen::MatrixXd R; // 观测噪声协方差矩阵
    Eigen::MatrixXd F; // 状态转移矩阵
    Eigen::MatrixXd B; // 控制输入矩阵
    Eigen::MatrixXd S; // 创新协方差矩阵
    Eigen::MatrixXd K; // 卡尔曼增益
    Eigen::MatrixXd I; // 单位矩阵

    Eigen::VectorXd X_pred;
    Eigen::VectorXd X_prev;
    Eigen::VectorXd target_state; // 目标状态
};


#endif // SPINTOP_PREDICTOR_HPP