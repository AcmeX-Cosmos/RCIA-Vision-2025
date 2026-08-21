// gimbal_control.hpp
#ifndef GIMBAL_CONTROL_HPP_
#define GIMBAL_CONTROL_HPP_

#include "Type.hpp"

#include <tf2_ros/buffer.h>

#include "trajectory_compensator.hpp"

namespace rcia {
namespace vision_tracker {
class GimbalControl {
public:
    explicit GimbalControl(std::weak_ptr<rclcpp::Node> node);

    rcia_vision_interfaces::msg::SerialTransmitData solve(const rcia_vision_interfaces::msg::TargetSpinTop &spinTop_target_msg,
                                        const rclcpp::Time &current_time,
                                        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_);

    ~GimbalControl() = default;

    enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 } state;

    std::vector<std::pair<double, double>> getTrajectory() const noexcept; 

    double current_flight_time_ = 0.0;
    double current_compen_height_ = 0.0;
    double current_distance_ = 0.0;

    // fire control
    rclcpp::Time last_fire_time_;
    uint8_t last_fire_flag_ = 0x00;

    std::map<int, double> ballistic_coefficients;
    
protected:
    // Get the armor positions from the target robot
    std::vector<Eigen::Vector3d> getArmorPositions(const Eigen::Vector3d &target_center,
                                                    const double yaw,
                                                    const double r1,
                                                    const double r2,
                                                    const double dz,
                                                    const size_t armors_num) const noexcept;

    // Select the best armor to shoot
    int selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                        const Eigen::Vector3d &target_center,
                        const double target_yaw,
                        const double target_v_yaw,
                        const size_t armors_num) const noexcept;

    void calcYawAndPitch(const Eigen::Vector3d &p,
                        const std::array<double, 3> rpy,
                        double &yaw,
                        double &pitch);

    uint8_t isOnTarget(const double cur_yaw,
                    const double cur_pitch,
                    const double target_yaw,
                    const double target_pitch,
                    const double distance) const noexcept;

    double getFlyingTime(const Eigen::Vector3d target_position);

    bool compensate(const Eigen::Vector3d &target_position, double &pitch) const noexcept ;

    double calculateTrajectory(const double x, const double angle) const noexcept ;

private:
    std::array<double, 3> rpy_;

    double prediction_delay_;
    double controller_delay_;

    double shooting_range_w_;
    double shooting_range_h_;

    double max_tracking_w_yaw_;
    int overflow_count_;
    int transfer_thresh_;

    double side_angle_;
    double min_switching_v_yaw_;
    double fire_time_interval_;

    double pitch_offset_;
    double yaw_offset_;
    
    std::weak_ptr<rclcpp::Node> node_;

    uint8_t last_fire_flag;

    std::unique_ptr<Trajectory_compensator> trajectory_compensator_;

};
}
}


#endif  // GIMBAL_CONTROL_HPP_
