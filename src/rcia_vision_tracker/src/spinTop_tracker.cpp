#include "spinTop_tracker.hpp"

namespace rcia {
namespace vision_tracker {
SpinTopTracker::SpinTopTracker(std::weak_ptr<rclcpp::Node> n) : 
        node_(n),
        measurement(Eigen::VectorXd::Zero(4)), 
        target_state(Eigen::VectorXd::Zero(9))
    {   
        auto node_ptr = node_.lock();
        max_match_distance_ = node_ptr->declare_parameter("tracker.max_match_distance", 0.2);
        max_match_yaw_diff_ = node_ptr->declare_parameter("tracker.max_match_yaw_diff", 1.0);
        tracking_thresh = node_ptr->declare_parameter("tracker.tracking_thresh", 10);

        ekf_ = std::make_unique<ExtendedKalmanFilter>();
        ekf_->r_x_ = node_ptr->declare_parameter("ekf.r_x", 0.15);
        ekf_->r_y_ = node_ptr->declare_parameter("ekf.r_y", 0.20);
        ekf_->r_z_ = node_ptr->declare_parameter("ekf.r_z", 0.25);
        ekf_->r_yaw_ = node_ptr->declare_parameter("ekf.r_yaw", 0.02);
        
        ekf_->s2q_x_ = node_ptr->declare_parameter("ekf.s2q_x", 15.0);
        ekf_->s2q_y_ = node_ptr->declare_parameter("ekf.s2q_y", 10.0);
        ekf_->s2q_z_ = node_ptr->declare_parameter("ekf.s2q_z", 5.00);
        ekf_->s2qyaw_ = node_ptr->declare_parameter("ekf.s2qyaw", 100.0);
        ekf_->s2qr_ = node_ptr->declare_parameter("ekf.s2qr", 80.0);

        tracked_id = 0;
        tracker_state = PATROL;

        node_ptr.reset();
    }


void SpinTopTracker::init(const rcia_vision_interfaces::msg::ArmorBaposeInfo::SharedPtr &armor_bapose_msg_, rcia_vision_interfaces::msg::OdomMeasurement odom_measurement_msg_) {
    
    if (armor_bapose_msg_->armor_type != "null") {
        tracked_armor = odom_measurement_msg_;
        tracked_id = armor_bapose_msg_->armor_pattern_idx;

        tracker_state = DETECTING;

        // 判断是否为前哨站
        tracked_armors_count = (tracked_id == 6) ? ArmorsCount::OUTPOST_3 : ArmorsCount::NORMAL_4;

        initEkf(tracked_armor);
    }
}

void SpinTopTracker::initEkf(rcia_vision_interfaces::msg::OdomMeasurement measurement) {
    double xa = measurement.pose.position.x;
    double ya = measurement.pose.position.y;
    double za = measurement.pose.position.z;
    last_yaw_ = 0;
    double yaw = orientationToYaw(measurement.pose.orientation);

    // Set initial position at 0.25m behind the target
    target_state = Eigen::VectorXd::Zero(9);
    double r = 0.25;
    double xc = xa + r * cos(yaw);
    double yc = ya + r * sin(yaw);
    dz = 0, another_r = r;
    target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;
  
    ekf_->setState(target_state);
}

void SpinTopTracker::update_dt() {
    dt_ = (current_time_ - previous_time_).seconds();
}


void SpinTopTracker::update(const rcia_vision_interfaces::msg::ArmorBaposeInfo::SharedPtr &armor_bapose_msg_, rcia_vision_interfaces::msg::OdomMeasurement odom_measurement_msg_) {
    // Get newest parameters
    try {
        auto node_ptr = node_.lock();
        tracking_thresh = node_ptr->get_parameter("tracker.tracking_thresh").as_int();

        max_match_distance_ = node_ptr->get_parameter("tracker.max_match_distance").as_double();
        max_match_yaw_diff_ = node_ptr->get_parameter("tracker.max_match_yaw_diff").as_double();

        ekf_->r_x_ = node_ptr->get_parameter("ekf.r_x").as_double();
        ekf_->r_y_ = node_ptr->get_parameter("ekf.r_y").as_double();
        ekf_->r_z_ = node_ptr->get_parameter("ekf.r_z").as_double();
        ekf_->r_yaw_ = node_ptr->get_parameter("ekf.r_yaw").as_double();
        
        ekf_->s2q_x_ = node_ptr->get_parameter("ekf.s2q_x").as_double();
        ekf_->s2q_y_ = node_ptr->get_parameter("ekf.s2q_y").as_double();
        ekf_->s2q_z_ = node_ptr->get_parameter("ekf.s2q_z").as_double();
        ekf_->s2qyaw_ = node_ptr->get_parameter("ekf.s2qyaw").as_double();
        ekf_->s2qr_ = node_ptr->get_parameter("ekf.s2qr").as_double();

        node_ptr.reset();
    } catch (const std::runtime_error &e) {
        cout << "spinTop_tracker" << ": " << e.what() << endl;
    }

    Eigen::VectorXd ekf_prediction = ekf_->exkalman_predict(dt_);

    bool matched = false;

    target_state = ekf_prediction;

    if (armor_bapose_msg_->armor_type != "null") {
        double min_position_diff = DBL_MAX;
        double yaw_diff = DBL_MAX;
        rcia_vision_interfaces::msg::OdomMeasurement same_id_armor;
        int same_id_armors_count = 0;
        auto predicted_position = ekf_->getArmorPositionFromState(ekf_prediction);

        // 如果遍历多个装甲板
        // for
        if (armor_bapose_msg_->armor_pattern_idx == tracked_id) {
            same_id_armor = odom_measurement_msg_;
            same_id_armors_count++;

            auto p = odom_measurement_msg_.pose.position;
            Eigen::Vector3d position_vec(p.x, p.y, p.z);
            double position_diff = (predicted_position - position_vec).norm();

            if (position_diff < min_position_diff) {
                min_position_diff = position_diff;
                yaw_diff = abs(orientationToYaw(odom_measurement_msg_.pose.orientation) - ekf_prediction(6));
                
                tracked_armor = odom_measurement_msg_;
                
                tracked_id = armor_bapose_msg_->armor_pattern_idx;

                tracked_armors_count = (tracked_id == 6) ? ArmorsCount::OUTPOST_3 : ArmorsCount::NORMAL_4;
            }
        }

        info_position_diff = min_position_diff;
        info_yaw_diff = yaw_diff;

        if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
            matched = true;
            auto p = tracked_armor.pose.position;

            double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
            measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
            target_state = ekf_->exkalman_update(measurement);
        } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
            handleArmorJump(same_id_armor);

        } else {
            // cout << "armor_solver No matched armor found!" << endl;
        }
    }

    if (target_state(8) < 0.12) {
        target_state(8) = 0.12;
        ekf_->setState(target_state);
    } else if (target_state(8) > 0.35) {
        target_state(8) = 0.35;
        ekf_->setState(target_state);
    }
    
    // cout << "detection_count_ " << detection_count_ << endl;
    // cout << "tracking_thresh" << tracking_thresh << endl;
    // cout << (detection_count_ > tracking_thresh) << endl;
    // cout << "lost_count_" << lost_count_ << endl;
    if (tracker_state == DETECTING) {
        if (matched) {
            detection_count_++;
            if (detection_count_ > tracking_thresh) {
                detection_count_ = 0;
                tracker_state = TRACKING;
                // std::cout << "armor_solver Tracker state: TRACKING " << tracked_id << std::endl;
            }
        } else {
            detection_count_ = 0;
            tracker_state = PATROL;
            // std::cout << "armor_solver Tracker state: PATROL " << tracked_id << std::endl;
        }
        } 
        
    else if (tracker_state == TRACKING) {
        if (!matched) {
            tracker_state = TEMP_LOST;
            lost_count_++;
            // std::cout << "armor_solver Tracker state: TEMP_LOST " << tracked_id << std::endl;
        }
    } 
    else if (tracker_state == TEMP_LOST) {
        if (!matched) {
            lost_count_++;
        if (lost_count_ > lost_thres) {
            lost_count_ = 0;
            tracker_state = PATROL;
            // std::cout << "armor_solver Tracker state: PATROL " << tracked_id << std::endl;
        }
    } else {
        tracker_state = TRACKING;
        lost_count_ = 0;
        // std::cout << "armor_solver Tracker state: TRACKING " << tracked_id << std::endl;
    }
}
}


double SpinTopTracker::orientationToYaw(const geometry_msgs::msg::Quaternion &q) {
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    // Make yaw change continuous (-pi~pi to -inf~inf)
    yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);

    last_yaw_ = yaw;

    return yaw;
}

void SpinTopTracker::handleArmorJump(const rcia_vision_interfaces::msg::OdomMeasurement &current_armor) {
    double last_yaw = target_state(6);
    double yaw = orientationToYaw(current_armor.pose.orientation);
  
    if (abs(yaw - last_yaw) > 0.4) {
        target_state(6) = yaw;
        // Only 4 armors has 2 radius and height
        if (tracked_armors_count == ArmorsCount::NORMAL_4) {
            dz = target_state(4) - current_armor.pose.position.z;
            target_state(4) = current_armor.pose.position.z;
            std::swap(target_state(8), another_r);
        }
        // cout << "armor_solver Armor Jump!" << endl;
    }
  
    auto p = current_armor.pose.position;
    Eigen::Vector3d current_p(p.x, p.y, p.z);
    Eigen::Vector3d infer_p = ekf_->getArmorPositionFromState(target_state);
  
    if ((current_p - infer_p).norm() > max_match_distance_) {
        //如果当前装甲板和推断装甲板之间的距离太大，状态错误，重置中心位置和速度状态
        double r = target_state(8);
        target_state(0) = p.x + r * cos(yaw);  // xc
        target_state(1) = 0;                   // vxc
        target_state(2) = p.y + r * sin(yaw);  // yc
        target_state(3) = 0;                   // vyc
        target_state(4) = p.z;                 // xz
        target_state(5) = 0;                   // vza
        cout << "armor_solver State wrong!" << endl;
    }
  
        ekf_->setState(target_state);
    }


}  // namespace vision_tracker
}  // namespace rcia