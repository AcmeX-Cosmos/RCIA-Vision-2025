#include "gimbal_control.hpp"

// #include <Eigen/Geometry>

namespace rcia {
namespace vision_tracker {

GimbalControl::GimbalControl(std::weak_ptr<rclcpp::Node> n) : node_(n) {
    // 初始化参数，可以从node中读取
    auto node_ptr = node_.lock();

    shooting_range_w_ = node_ptr->declare_parameter("gimbal_control.shooting_range_width", 0.135);
    shooting_range_h_ = node_ptr->declare_parameter("gimbal_control.shooting_range_height", 0.135);
    max_tracking_w_yaw_ = node_ptr->declare_parameter("gimbal_control.max_tracking_w_yaw", 6.0);
    prediction_delay_ = node_ptr->declare_parameter("gimbal_control.prediction_delay", 0.05);
    controller_delay_ = node_ptr->declare_parameter("gimbal_control.controller_delay", 0.05);
    side_angle_ = node_ptr->declare_parameter("gimbal_control.side_angle", 15.0);
    min_switching_v_yaw_ = node_ptr->declare_parameter("gimbal_control.min_switching_v_yaw", 1.0);
    fire_time_interval_ = node_ptr->declare_parameter("gimbal_control.fire_time_interval", 0.2);

    pitch_offset_ = node_ptr->declare_parameter("compensator.pitch_offset", 0.0);
    yaw_offset_ = node_ptr->declare_parameter("compensator.yaw_offset", -0.025);
    
    trajectory_compensator_ = std::make_unique<Trajectory_compensator>();
    trajectory_compensator_->initial_velocity = node_ptr->declare_parameter("compensator.initial_velocity", 23.0);
    trajectory_compensator_->bullet_mass = node_ptr->declare_parameter("compensator.bullet_mass", 0.003);
    node_ptr->declare_parameter("ballistic_coefficients.bc_keys",  std::vector<int>{0 , 1  ,  2   , 3    ,4     ,     5  ,   6   ,  7    ,    8   ,  9   });
    node_ptr->declare_parameter("ballistic_coefficients.bc_values", std::vector<double>{0.45, 0.355, 0.245, 0.175, 0.125 , 0.085 , 0.055 , 0.038 , 0.0235 ,0.0185});

    // trajectory_compensator_->ballistic_coefficient = node_ptr->declare_parameter("compensator.ballistic_coefficient", 0.25);

    state = State::TRACKING_ARMOR;
    overflow_count_ = 0;
    transfer_thresh_ = 5;

    last_fire_time_ = node_ptr->now();

    node_ptr.reset();
}

rcia_vision_interfaces::msg::SerialTransmitData GimbalControl::solve(
    const rcia_vision_interfaces::msg::TargetSpinTop &spinTop_target_msg,
    const rclcpp::Time &current_time,
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) { 

    // Get newest parameters
    try {
        auto node_ptr = node_.lock();
        max_tracking_w_yaw_ = node_ptr->get_parameter("gimbal_control.max_tracking_w_yaw").as_double();
        prediction_delay_ = node_ptr->get_parameter("gimbal_control.prediction_delay").as_double();
        controller_delay_ = node_ptr->get_parameter("gimbal_control.controller_delay").as_double();
        side_angle_ = node_ptr->get_parameter("gimbal_control.side_angle").as_double();
        min_switching_v_yaw_ = node_ptr->get_parameter("gimbal_control.min_switching_v_yaw").as_double();
        fire_time_interval_ = node_ptr->get_parameter("gimbal_control.fire_time_interval").as_double();

        pitch_offset_ = node_ptr->get_parameter("compensator.pitch_offset").as_double();
        yaw_offset_ = node_ptr->get_parameter("compensator.yaw_offset").as_double();


        trajectory_compensator_->keys = node_ptr->get_parameter("ballistic_coefficients.bc_keys").as_integer_array();
        trajectory_compensator_->values = node_ptr->get_parameter("ballistic_coefficients.bc_values").as_double_array();
       
        trajectory_compensator_->initial_velocity = node_ptr->get_parameter("compensator.initial_velocity").as_double();
        trajectory_compensator_->bullet_mass = node_ptr->get_parameter("compensator.bullet_mass").as_double();
        // trajectory_compensator_->ballistic_coefficient = node_ptr->get_parameter("compensator.ballistic_coefficient").as_double();

        node_ptr.reset();
    } catch (const std::runtime_error &e) {
        cout << "gimbal_control" << ": " << e.what() << endl;
    }

    // Get current roll, yaw and pitch of gimbal
    try {
        auto gimbal_tf =
        tf2_buffer_->lookupTransform(spinTop_target_msg.header.frame_id, "gimbal_link", tf2::TimePointZero);
        auto msg_q = gimbal_tf.transform.rotation;

        tf2::Quaternion tf_q;
        tf2::fromMsg(msg_q, tf_q);
        tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
        rpy_[1] = -rpy_[1];
    } catch (tf2::TransformException &ex) {
        cout << "gimbal_control" << ": " << ex.what() << endl;
        throw ex;
    }

    // Use flying time to approximately predict the position of spinTop_target_msg
    Eigen::Vector3d target_position(spinTop_target_msg.position.x, spinTop_target_msg.position.y, spinTop_target_msg.position.z);
    double spinTop_target_msg_yaw = spinTop_target_msg.yaw;
    this->current_flight_time_ = getFlyingTime(target_position);

    double dt = (current_time - rclcpp::Time(spinTop_target_msg.header.stamp)).seconds() + this->current_flight_time_ + prediction_delay_;
    target_position.x() += dt * spinTop_target_msg.velocity.x;
    target_position.y() += dt * spinTop_target_msg.velocity.y;
    target_position.z() += dt * spinTop_target_msg.velocity.z;
    spinTop_target_msg_yaw += dt * spinTop_target_msg.w_yaw;

    // Choose the best armor to shoot
    std::vector<Eigen::Vector3d> armor_positions = getArmorPositions(
        target_position, spinTop_target_msg_yaw, spinTop_target_msg.r1, spinTop_target_msg.r2, spinTop_target_msg.dz, spinTop_target_msg.armors_count);
    int idx =
        selectBestArmor(armor_positions, target_position, spinTop_target_msg_yaw, spinTop_target_msg.w_yaw, spinTop_target_msg.armors_count);
    auto chosen_armor_position = armor_positions.at(idx);
    if (chosen_armor_position.norm() < 0.1) {
        throw std::runtime_error("No valid armor to shoot");
    }

    // Calculate yaw, pitch, distance
    double yaw, pitch;
    calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
    double distance = chosen_armor_position.norm();

    // Initialize gimbal_cmd
    rcia_vision_interfaces::msg::SerialTransmitData vision_msg;
    vision_msg.header = spinTop_target_msg.header;
    vision_msg.distance = distance;
    vision_msg.fire_flag = isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);
    
    switch (state) {
        case TRACKING_ARMOR: {
        if (std::abs(spinTop_target_msg.w_yaw) > max_tracking_w_yaw_) {
            overflow_count_++;
        } else {
            overflow_count_ = 0;
        }

        if (overflow_count_ > transfer_thresh_) {
            state = TRACKING_CENTER;
        }

        // If isOnspinTop_target_msg() never returns true, adjust controller_delay to force the gimbal to move
        if (controller_delay_ != 0) {
            target_position.x() += controller_delay_ * spinTop_target_msg.velocity.x;
            target_position.y() += controller_delay_ * spinTop_target_msg.velocity.y;
            target_position.z() += controller_delay_ * spinTop_target_msg.velocity.z;
            spinTop_target_msg_yaw += controller_delay_ * spinTop_target_msg.w_yaw;
            armor_positions = getArmorPositions(target_position,
                                                spinTop_target_msg_yaw,
                                                spinTop_target_msg.r1,
                                                spinTop_target_msg.r2,
                                                spinTop_target_msg.dz,
                                                spinTop_target_msg.armors_count);

            chosen_armor_position = armor_positions.at(idx);
            vision_msg.distance = chosen_armor_position.norm();
            if (chosen_armor_position.norm() < 0.1) {
            throw std::runtime_error("No valid armor to shoot");
            }
            calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
        }
        break;
        }
        case TRACKING_CENTER: {
            if (std::abs(spinTop_target_msg.w_yaw) < max_tracking_w_yaw_) {
                overflow_count_++;
            } else {
                overflow_count_ = 0;
            }

            if (overflow_count_ > transfer_thresh_) {
                state = TRACKING_ARMOR;
                overflow_count_ = 0;
            }
            
            vision_msg.fire_flag = 0x01;
            calcYawAndPitch(target_position, rpy_, yaw, pitch);
            break;
        }
    }

    // vision_msg.yaw = yaw * 180 / M_PI;
    // vision_msg.pitch = pitch * 180 / M_PI;
    // cout << " pitch: " << pitch * 180 / M_PI * 100.0 << endl;
    // cout << " rpy_[1]: " << rpy_[1] * 180 / M_PI * 100.0 << endl;
    vision_msg.yaw_angle = (yaw - rpy_[2] + yaw_offset_) * 180 / M_PI * 100.0;
    vision_msg.pitch_angle = (pitch - rpy_[1] + pitch_offset_) * 180 / M_PI * 100.0;
    vision_msg.find_flag = 0x01;
    
    double time_since_last = (current_time - last_fire_time_).seconds();
    if (time_since_last >= fire_time_interval_) {
        if (vision_msg.fire_flag) {
            last_fire_time_ = current_time;
            last_fire_flag_ = 0x01;
        } else {
            last_fire_flag_ = 0x00;
        }
    } else {
        vision_msg.fire_flag = 0x00;
        last_fire_flag_ = 0x00;
    }

    if (vision_msg.fire_flag) {
        // cout << "fire !!!" << endl;
    }
    last_fire_flag = vision_msg.fire_flag;

    return vision_msg;                                        
}





uint8_t GimbalControl::isOnTarget(const double cur_yaw, const double cur_pitch,
                        const double target_yaw, const double target_pitch,
                        const double distance) const noexcept {
    // Judge whether to shoot
    double shooting_range_yaw = std::abs(atan2(shooting_range_w_ / 2, distance));
    double shooting_range_pitch = std::abs(atan2(shooting_range_h_ / 2, distance));
    // Limit the shooting area to 1 degree to avoid not shooting when distance is too large
    shooting_range_yaw = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
    shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
    // if (std::abs(cur_yaw - target_yaw) < shooting_range_yaw) {
    //     return 0x01;
    // }
    if (std::abs(cur_yaw - target_yaw) < shooting_range_yaw &&
        std::abs(cur_pitch - target_pitch) < shooting_range_pitch) {
        return 0x01;
    }

  return 0x00;
}

std::vector<Eigen::Vector3d> GimbalControl::getArmorPositions(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double dz,
                                                       const size_t armors_num) const noexcept {
    auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
    // Calculate the position of each armor
    bool is_current_pair = true;
    double r = 0., target_dz = 0.;
    for (size_t i = 0; i < armors_num; i++) {
        double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
        if (armors_num == 4) {
            r = is_current_pair ? r1 : r2;
            target_dz = is_current_pair ? 0 : dz;
            is_current_pair = !is_current_pair;
        } else {
            r = r1;
            target_dz = dz;
        }
        armor_positions[i] =
        target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
    }
    return armor_positions;
}

int GimbalControl::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const Eigen::Vector3d &target_center,
                            const double target_yaw,
                            const double target_v_yaw,
                            const size_t armors_num) const noexcept {
    // Angle between the car's center and the X-axis
    double alpha = std::atan2(target_center.y(), target_center.x());
    // Angle between the front of observed armor and the X-axis
    double beta = target_yaw;

    // clang-format off
    Eigen::Matrix2d R_odom2center;
    Eigen::Matrix2d R_odom2armor;
    R_odom2center << std::cos(alpha), std::sin(alpha), 
                    -std::sin(alpha), std::cos(alpha);
    R_odom2armor << std::cos(beta), std::sin(beta), 
                    -std::sin(beta), std::cos(beta);
    // clang-format on
    Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

    // Equal to (beta - alpha) in most cases
    double decision_angle = -std::asin(R_center2armor(0, 1));

    // Angle thresh of the armor jump
    double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;

    // Avoid the frequent switch between two armor
    if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
        theta = 0;
    }

    double temp_angle = decision_angle + M_PI / armors_num - theta;

    if (temp_angle < 0) {
        temp_angle += 2 * M_PI;
    }

    int selected_id = static_cast<int>(temp_angle / (2 * M_PI / armors_num));
    return selected_id;
}

void GimbalControl::calcYawAndPitch(const Eigen::Vector3d &p,
                             const std::array<double, 3> rpy,
                             double &yaw,
                             double &pitch) {
    // Calculate yaw and pitch
    yaw = atan2(p.y(), p.x());
    pitch = atan2(p.z(), p.head(2).norm());

    trajectory_compensator_->angle_compensator(p, pitch, this->current_compen_height_, this->current_distance_);
    // cout << "p.z()" << p.z() << "\t" << "p.head(2).norm() " << p.head(2).norm() << endl;
}

/**
 * @brief 计算子弹飞行时间
 * 
 * 该函数根据目标位置、距离和子弹速度，计算子弹从发射到命中目标所需的飞行时间。
 * 
 * @param target_position 目标位置矩阵，包含目标的三维坐标
 * @param distance 目标距离（单位：米）
 * @param velocity 子弹速度（单位：米/秒）
 * @return 子弹飞行时间（单位：秒）
 */
double GimbalControl::getFlyingTime(const Eigen::Vector3d target_position) {
    double distance =
        sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));

    // 计算子弹与目标之间的角度
    double angle = atan2(target_position(2), distance);
    
    // 临时给弹速 26 
    double bullet_speed = 26;

    // 计算子弹飞行时间 
    double t = distance / (bullet_speed * cos(angle));

    return t;
}

}
}