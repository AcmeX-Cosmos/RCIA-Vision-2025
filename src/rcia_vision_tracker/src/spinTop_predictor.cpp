#include "spinTop_predictor.hpp"

void ExtendedKalmanFilter::update_R(const VectorXd &Z) {
    double r_x = this->r_x_;
    double r_y = this->r_y_;
    double r_z = this->r_z_;
    double r_yaw = this->r_yaw_;

    this->R.diagonal() << abs(r_x * Z[0]), abs(r_y * Z[1]), abs(r_z * Z[2]), r_yaw;
}


void ExtendedKalmanFilter::update_Q(double pdt_) {
    double t = pdt_;
    double t_sq = std::pow(t, 2);
    double t_cube_over_3 = std::pow(t, 3) / 3;
    double t_quart_over_4 = std::pow(t, 4) / 4;

    // 定义噪声强度参数                     
    double s2q_x = this->s2q_x_;                          // x位置噪声强度
    double s2q_y = this->s2q_y_;                          // y位置噪声强度 
    double s2q_z = this->s2q_z_;                          // z位置噪声强度
    double s2q_yaw = this->s2qyaw_;                       // yaw噪声强度
    double s2q_w_yaw = this->s2qyaw_ / t_sq;              // 方向角噪声强度
    double s2qr = this->s2qr_;

    // 位置和速度的协方差
    this->Q.block<2, 2>(0, 0) << t_quart_over_4 * s2q_x, t_cube_over_3 * s2q_x,
                           t_cube_over_3 * s2q_x, t_sq * s2q_x;         // X 轴

    this->Q.block<2, 2>(2, 2) << t_quart_over_4 * s2q_y, t_cube_over_3 * s2q_y,
                           t_cube_over_3 * s2q_y, t_sq * s2q_y;         // Y 轴

    this->Q.block<2, 2>(4, 4) << t_quart_over_4 * s2q_z, t_cube_over_3 * s2q_z,
                           t_cube_over_3 * s2q_z, t_sq * s2q_z;         // Z 轴

    // 方向角和角速度的协方差
    this->Q.block<2, 2>(6, 6) << t_quart_over_4 * s2q_yaw, t_cube_over_3 * s2q_yaw,
                           t_cube_over_3 * s2q_yaw, t_sq * s2q_yaw;

    // 半径的协方差
    this->Q(8, 8) = t_quart_over_4 * s2qr;

    // cout << "Q" << this->Q << endl;
}


VectorXd ExtendedKalmanFilter::h(const VectorXd & x) {
    VectorXd z(4);
    double cx = x(0), cy = x(2), cz = x(4), yaw = x(6), r = x(8);

    z(0) = cx - r * cos(yaw);       // ax
    z(1) = cy - r * sin(yaw);       // ay
    z(2) = cz;                      // az
    z(3) = yaw;                     // phi_

    // cout << "\n\n" << cx << "\t"<< r << "\t" << phi_ << endl;
    // cout << "\nz" << z << endl;
    return z;
}
MatrixXd ExtendedKalmanFilter::jacobian_h(const VectorXd & x) {
    MatrixXd J_h(4, 9);
    J_h.setZero();
    double yaw = x(6), r = x(8);

    //      xc   v_xc yc   v_yc za   v_za   yaw            v_yaw     r
    J_h <<   1,   0,   0,   0,   0,   0,    r * sin(yaw) ,  0,      -cos(yaw),  // cos(phi_)
             0,   0,   1,   0,   0,   0,   -r * cos(yaw),   0,      -sin(yaw),  // sin(phi_)
             0,   0,   0,   0,   1,   0,    0,              0,       0,
             0,   0,   0,   0,   0,   0,    1,              0,       0;
    
    return J_h;
};

// 状态 转移 函数
VectorXd ExtendedKalmanFilter::f(const VectorXd& X_prev, double pdt_) {
    VectorXd x_new = X_prev;
    
    // cout << "f X_prev(7) " << X_prev(7) << endl;
    x_new(0) += X_prev(1) * pdt_;
    x_new(2) += X_prev(3) * pdt_;
    x_new(4) += X_prev(5) * pdt_;
    x_new(6) += X_prev(7) * pdt_;
    // cout << "x_pred(7): " << X_prev(7) / 3.1415926 << endl;

    return x_new;
}

MatrixXd ExtendedKalmanFilter::jacobian_f(double pdt_) {
    // 初始化雅可比矩阵
    MatrixXd F(9, 9);
    F.setZero(); 

    // double t_sq = 0.5 * std::pow(pdt_, 2);
    
    F(0, 0) = F(1, 1) = F(2, 2) = F(3, 3) = F(4, 4) = F(5, 5) = F(6, 6) = F(7, 7) = F(8, 8) = 1;

    F(0, 1) = F(2, 3) = F(4, 5) = F(6, 7) = pdt_;
    
    // cout << "F \n" << F << endl;
    return F;
}

Eigen::Vector3d ExtendedKalmanFilter::getArmorPositionFromState(const Eigen::VectorXd &x) noexcept {
    double xc = x(0), yc = x(2), za = x(4);
    double yaw = x(6), r = x(8);

    double xa = xc - r * cos(yaw);
    double ya = yc - r * sin(yaw);

    return Eigen::Vector3d(xa, ya, za);
}


double get_current_timestamp() {
    auto now = chrono::steady_clock::now();

    auto duration_since_epoch = now.time_since_epoch();

    double timestamp = static_cast<double>(
        chrono::duration_cast<chrono::milliseconds>(duration_since_epoch).count());

    return timestamp;
}


void ExtendedKalmanFilter::setState(const Eigen::VectorXd &x) noexcept { 
    this->X_prev = x;
}

MatrixXd ExtendedKalmanFilter::exkalman_predict(const double &pdt_)
{
    // cout << "# ----------------------- # EKFST # ----------------------- # " << endl;

    F = jacobian_f(pdt_);

    this->X_pred = f(this->X_prev, pdt_);

    update_Q(pdt_);

    this->P = F * this->P * F.transpose() + this->Q;
    
    this->X_prev = this->X_pred;

    return this->X_pred;
}

MatrixXd ExtendedKalmanFilter::exkalman_update(const VectorXd &Z)
{

    update_R(Z);

    H = jacobian_h(this->X_pred);

    S = H * this->P * H.transpose() + R;

    if (S.determinant() == 0 || std::isnan(S.determinant())) {
        std::cout << "S matrix is singular or contains NaN." << std::endl;
        K = this->P * H.transpose() * S.completeOrthogonalDecomposition().pseudoInverse();
    } else {
        K = this->P * H.transpose() * S.inverse();
    }

    VectorXd smallh = h(this->X_pred);

    this->X_prev = this->X_pred + K * (Z - smallh);

    this->P = (I - K * H) * this->P;

    return this->X_prev;

}



ExtendedKalmanFilter::ExtendedKalmanFilter() :
    X_prev(9),  X_pred(9),
    P(9, 9), Q(9, 9), H(4, 9), R(4, 4), F(9, 9), I(9, 9)

    {   
        X_prev.setZero();
        X_pred.setZero();
        
        P.setZero();
        initializeQ();
        H.setIdentity();
        initializeR();
        F.setIdentity();
        I.setIdentity();
    }


// // +----------------------------------------------------------------------------------------------------------------+
// // # -------------------------------------- #  ExtendedKalmanFilter::initializeR # -------------------------------------- # 
// // # -------------------------------------------- # 测量噪声协方差 ( R ) # ------------------------------------------ #
// // # ------------------------------------------ # R 越小，越相信相机的测量值 # ---------------------------------------- #
// // # ----------------------------------- # 较大的 ( R ) 值表示测量数据的不准确性高 # ----------------------------------- #
// // # ------------------------------------ # 滤波器会更多地信任模型预测而非测量值 # -------------------------------------- #
// // # -------------------------------------- #  ExtendedKalmanFilter::initializeR # -------------------------------------- # 
// // +----------------------------------------------------------------------------------------------------------------+
void ExtendedKalmanFilter::initializeR() {
    R.setIdentity();
    // R << 0.005, 0.000, 0.000, 0.000,                                                    // whole_vehicle.cx,
    //      0.000, 0.005, 0.000, 0.000,                                                    // whole_vehicle.cy,
    //      0.000, 0.000, 0.005, 0.000,                                                    // whole_vehicle.cz,
    //      0.000, 0.000, 0.000, 0.035;                                                    // whole_vehicle.yaw;
}

// // +----------------------------------------------------------------------------------------------------------------+
// // # -------------------------------------- #  ExtendedKalmanFilter::initializeQ # -------------------------------------- # 
// // # -------------------------------------------- # 过程噪声协方差 ( Q ) # ------------------------------------------ #
// // # ------------------------------------------ # Q 越小，越相信模型预测值 # ----------------------------------------- #
// // # --------------------------------- # 较大的 ( Q ) 值表示模型预测中有较高的不确定性 # -------------------------------- #
// // # ----------------------------------- # 会使得滤波器更多地信任观测值而非预测值 # ------------------------------------- #
// // # -------------------------------------- #  ExtendedKalmanFilter::initializeQ # -------------------------------------- # 
// // +----------------------------------------------------------------------------------------------------------------+
void ExtendedKalmanFilter::initializeQ() {
    Q.setIdentity();
    // Q << 0.020, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,                              // whole_vehicle.cx,
    //      0.000, 0.020, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,                              // 
    //      0.000, 0.000, 0.020, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,                              // whole_vehicle.cy,
    //      0.000, 0.000, 0.000, 0.020, 0.000, 0.000, 0.000, 0.000, 0.000,                              // 
    //      0.000, 0.000, 0.000, 0.000, 0.020, 0.000, 0.000, 0.000, 0.000,                              // whole_vehicle.cz,
    //      0.000, 0.000, 0.000, 0.000, 0.000, 0.020, 0.000, 0.000, 0.000,                              // 
    //      0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.035, 0.000, 0.000,                              // whole_vehicle.yaw;
    //      0.000, 0.000, 0.000, 0.000, 0.000, 0.020, 0.000, 0.035, 0.000,                              // 
    //      0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.020, 0.000, 0.020;                              // whole_vehicle.r;
}