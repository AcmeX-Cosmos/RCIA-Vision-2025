// spinTop_tracker.hpp
#ifndef SPINTOP_TRACKER_HPP
#define SPINTOP_TRACKER_HPP

#include "Type.hpp"

#include "spinTop_predictor.hpp"

namespace rcia{
namespace vision_tracker{
enum class ArmorsCount { NORMAL_4 = 4, OUTPOST_3 = 3 };  // 装甲板个数

class SpinTopTracker{
public:
    explicit SpinTopTracker(std::weak_ptr<rclcpp::Node> node);
    
    ~SpinTopTracker() = default;

    void init(const rcia_vision_interfaces::msg::ArmorBaposeInfo::SharedPtr &armor_bapose_msg_, rcia_vision_interfaces::msg::OdomMeasurement odom_measurement_msg_);

    void update(const rcia_vision_interfaces::msg::ArmorBaposeInfo::SharedPtr &armor_bapose_msg_, rcia_vision_interfaces::msg::OdomMeasurement odom_measurement_msg_);

    enum State {
        PATROL,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    } tracker_state;

    uint8_t tracked_id;
    ArmorsCount tracked_armors_count;
    rcia_vision_interfaces::msg::OdomMeasurement tracked_armor;

    Eigen::VectorXd measurement;
    Eigen::VectorXd target_state;

    double max_match_distance_;
    double max_match_yaw_diff_;

    double info_position_diff;
    double info_yaw_diff;

    double dz, another_r;

    std::unique_ptr<ExtendedKalmanFilter> ekf_;

    int tracking_thresh;  // 帧
    int lost_thres;       // 秒

    // 时间戳
    rclcpp::Time current_time_;
    rclcpp::Time previous_time_;
    
    double dt_;
    void update_dt();       // 更新时间戳

    int detection_count_ = 0;
    int lost_count_ = 0;
    
protected:

    void initEkf(rcia_vision_interfaces::msg::OdomMeasurement measurement);

    double orientationToYaw(const geometry_msgs::msg::Quaternion &q);

    void handleArmorJump(const rcia_vision_interfaces::msg::OdomMeasurement &current_armor);

private:
    double last_yaw_;

    std::weak_ptr<rclcpp::Node> node_;
    
};
}
}

#endif // SPINTOP_TRACKER_HPP