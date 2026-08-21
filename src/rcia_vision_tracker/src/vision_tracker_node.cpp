// vision_tracker_node.cpp
#include "vision_tracker_node.hpp"

namespace rcia{
namespace vision_tracker{
    VisionTrackerNode::VisionTrackerNode(const rclcpp::NodeOptions &options)
    : Node("vision_tracker_node", options), control_(nullptr)
{   
    debug_mode_ = this->declare_parameter("debug", true);

    lost_time_thresh_ = this->declare_parameter("tracker.lost_time_thresh", 0.3);

    // 初始化tf2相关
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;

    armor_bapose_sub_.subscribe(this, "armor_bapose_info", qos_profile);
    target_frame_ = this->declare_parameter("gimbal_link", "odom");
    tf2_filter_ = std::make_shared<tf2_filter>(armor_bapose_sub_,
        *tf2_buffer_, target_frame_, 10,
        this->get_node_logging_interface(),
        this->get_node_clock_interface(),
        std::chrono::duration<int>(1));
    
    tf2_filter_->registerCallback(&VisionTrackerNode::solver_callback, this);

    // 待优化
    rclcpp::QoS qos_profile2(rclcpp::KeepLast(5));
    qos_profile2.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile2.history(rclcpp::HistoryPolicy::KeepLast);
    qos_profile2.durability(rclcpp::DurabilityPolicy::Volatile);

    serial_receive_sub_ = this->create_subscription<rcia_vision_interfaces::msg::SerialReceiveData>(
        "electrl_data", 10, std::bind(&VisionTrackerNode::serial_receive_callback, this, std::placeholders::_1));

    // 创建发布者
    spin_top_info_pub_ = this->create_publisher<rcia_vision_interfaces::msg::TargetSpinTop>("spin_top_topic", 2);
    vision_data_pub_ = this->create_publisher<rcia_vision_interfaces::msg::SerialTransmitData>("vision_data", 2);
    odom_measurement_pub_ = this->create_publisher<rcia_vision_interfaces::msg::OdomMeasurement>("odom_measurement", 2);
    tracker_state_pub_ = this->create_publisher<rcia_vision_interfaces::msg::TrackerState>("tracker_state", 2);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 2);

    // 可视化标记发布
    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;

    armors_marker_.ns = "filtered_armors";
    armors_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    armors_marker_.scale.x = armors_marker_.scale.y = armors_marker_.scale.z = 0.1;
    armors_marker_.color.a = 1.0;
    armors_marker_.color.r = 1.0;

}

// 此处可优化
void VisionTrackerNode::serial_receive_callback(std::shared_ptr<rcia_vision_interfaces::msg::SerialReceiveData> msg) {
    std::lock_guard<std::mutex> lock(serial_data_mutex_);
    latest_serial_data_ = *msg;
}

void VisionTrackerNode::solver_callback(const rcia_vision_interfaces::msg::ArmorBaposeInfo::SharedPtr &armor_bapose_msg_)
{   
    if (control_ == nullptr)    control_ = std::make_unique<GimbalControl>(weak_from_this());
    if (tracker_ == nullptr)    tracker_ = std::make_unique<SpinTopTracker>(weak_from_this());
    if (heartbeat_ == nullptr)  heartbeat_ = std::make_unique<HeartBeatPublisher>(weak_from_this());
    
    rcia_vision_interfaces::msg::OdomMeasurement odom_measurement_msg_;
    rcia_vision_interfaces::msg::TargetSpinTop spinTop_target_msg;

    // update current_time_
    rclcpp::Time time = armor_bapose_msg_->header.stamp;
    tracker_->current_time_ = time;
    spinTop_target_msg.header.stamp = time;

    spinTop_target_msg.header.frame_id = target_frame_;

    // 执行坐标变换
    armor_pose_transform(armor_bapose_msg_, odom_measurement_msg_);

    if (tracker_->tracker_state == SpinTopTracker::PATROL) {
        tracker_->init(armor_bapose_msg_, odom_measurement_msg_);
        spinTop_target_msg.tracking = false;
    } 
    else{
        tracker_->update_dt();
        tracker_->lost_thres = std::abs(static_cast<int>(lost_time_thresh_ / tracker_->dt_));
        tracker_->update(armor_bapose_msg_, odom_measurement_msg_);

        const auto &state = tracker_->target_state;
        if (tracker_->tracker_state == SpinTopTracker::PATROL) {
            tracker_->init(armor_bapose_msg_, odom_measurement_msg_);
            spinTop_target_msg.tracking = false;
        } 
        else{
            tracker_->update_dt();
            tracker_->lost_thres = std::abs(static_cast<int>(lost_time_thresh_ / tracker_->dt_));
            tracker_->update(armor_bapose_msg_, odom_measurement_msg_);
    
            if (tracker_->tracker_state == SpinTopTracker::DETECTING) {
                spinTop_target_msg.tracking = false;
            }
            else if (tracker_->tracker_state == SpinTopTracker::TRACKING ||
                tracker_->tracker_state == SpinTopTracker::TEMP_LOST) {
                spinTop_target_msg.tracking = true;
    
                const auto &state = tracker_->target_state;
                spinTop_target_msg.id = tracker_->tracked_id;
                spinTop_target_msg.armors_count = static_cast<int>(tracker_->tracked_armors_count);
                spinTop_target_msg.position.x = state(0);
                spinTop_target_msg.velocity.x = state(1);
                spinTop_target_msg.position.y = state(2);
                spinTop_target_msg.velocity.y = state(3);
                spinTop_target_msg.position.z = state(4);
                spinTop_target_msg.velocity.z = state(5);
                spinTop_target_msg.yaw = state(6);
                spinTop_target_msg.w_yaw = state(7);
                spinTop_target_msg.r1 = state(8);
                spinTop_target_msg.r2 = tracker_->another_r;
                spinTop_target_msg.dz = tracker_->dz;
                spinTop_target_msg.yaw_diff = tracker_->info_yaw_diff;
                spinTop_target_msg.position_diff = tracker_->info_position_diff;
                
            }
        }
    }

    // Solve gimbal control command
    if (spinTop_target_msg.tracking) {
        try {
            vision_control_msg_ = control_->solve(spinTop_target_msg, this->now(), tf2_buffer_);
        } catch (const std::exception& e) { // 捕获具体异常
            RCLCPP_ERROR(this->get_logger(), "Solver error: %s", e.what());
            vision_control_msg_.yaw_angle = 0;
            vision_control_msg_.pitch_angle = 0;
            vision_control_msg_.distance = -1;
            vision_control_msg_.fire_flag = 0x00;
            vision_control_msg_.find_flag = 0x00;
        }
    } else {
        vision_control_msg_.yaw_angle = 0;
        vision_control_msg_.pitch_angle = 0;
        vision_control_msg_.distance = -1;
        vision_control_msg_.fire_flag = 0x00;
        vision_control_msg_.find_flag = 0x00;
        }
    

    vision_control_msg_.time_stamp = latest_serial_data_.time_stamp;
    vision_control_msg_.operator_ui_x = 0;
    vision_control_msg_.operator_ui_y = 0;

    vision_data_pub_->publish(vision_control_msg_);
    
    publishMarkers(spinTop_target_msg);

    spinTop_target_msg.yaw = spinTop_target_msg.yaw * (180.0 / M_PI);
    // 发布 spinTop_target_msg
    spin_top_info_pub_->publish(spinTop_target_msg);

    publishTrackerState();

    tracker_->previous_time_ = time;

    heartbeat_->refresh(vision_control_msg_);
}




void VisionTrackerNode::armor_pose_transform(const rcia_vision_interfaces::msg::ArmorBaposeInfo::SharedPtr &armor_bapose_msg_, rcia_vision_interfaces::msg::OdomMeasurement &odom_measurement_msg_) {
    geometry_msgs::msg::PoseStamped pose_in;
    geometry_msgs::msg::PoseStamped pose_out;

    pose_in.header = armor_bapose_msg_->header;
    pose_in.pose = armor_bapose_msg_->pose;

    auto transform = tf2_buffer_->lookupTransform(
        "odom",
        pose_in.header.frame_id,
        tf2::TimePointZero); // 使用最新可用变换

    tf2::doTransform(pose_in, pose_out, transform);
    
    odom_measurement_msg_.pose = pose_out.pose;

    // 将四元数转换为欧拉角
    tf2::Quaternion q(
        odom_measurement_msg_.pose.orientation.x,
        odom_measurement_msg_.pose.orientation.y,
        odom_measurement_msg_.pose.orientation.z,
        odom_measurement_msg_.pose.orientation.w
    );
    
    tf2::Matrix3x3 rotation_matrix(q);
    double roll_rad, pitch_rad, yaw_rad;
    rotation_matrix.getRPY(roll_rad, pitch_rad, yaw_rad);

    // 将欧拉角从弧度制转换为角度制
    odom_measurement_msg_.euler.x = pitch_rad * (180.0 / M_PI);
    odom_measurement_msg_.euler.y = yaw_rad * (180.0 / M_PI);
    odom_measurement_msg_.euler.z = roll_rad * (180.0 / M_PI);

    odom_measurement_pub_->publish(odom_measurement_msg_);
}

void VisionTrackerNode::publishTrackerState() {

    rcia_vision_interfaces::msg::TrackerState tracker_state_msg;
    switch (tracker_->tracker_state) {
        case SpinTopTracker::PATROL:
            tracker_state_msg.state = "Patrol";
            break;
        case SpinTopTracker::DETECTING:
            tracker_state_msg.state = "Detecting";
            break;
        case SpinTopTracker::TRACKING:
            tracker_state_msg.state = "Tracking";
            break;
        case SpinTopTracker::TEMP_LOST:
            tracker_state_msg.state = "TempLost";
            break;
    }
    tracker_state_msg.header.stamp = this->now();

    tracker_state_msg.detection_count = tracker_->detection_count_;
    tracker_state_msg.lost_count = tracker_->lost_count_;
    tracker_state_msg.last_pitch = vision_control_msg_.pitch_angle;
    tracker_state_msg.last_yaw = vision_control_msg_.yaw_angle;

    tracker_state_msg.armor_distance = control_->current_distance_;
    tracker_state_msg.flight_time = control_->current_flight_time_;
    tracker_state_msg.compen_height = control_->current_compen_height_;

    tracker_state_pub_->publish(tracker_state_msg);
}

void VisionTrackerNode::publishMarkers(const rcia_vision_interfaces::msg::TargetSpinTop &spinTop_target_msg) {
    position_marker_.header = spinTop_target_msg.header;
    armors_marker_.header = spinTop_target_msg.header;

    if (spinTop_target_msg.tracking) {
        double yaw = spinTop_target_msg.yaw, r1 = spinTop_target_msg.r1, r2 = spinTop_target_msg.r2;
        double xc = spinTop_target_msg.position.x, yc = spinTop_target_msg.position.y, za = spinTop_target_msg.position.z;
        // double vx = spinTop_target_msg.velocity.x, vy = spinTop_target_msg.velocity.y, vz = spinTop_target_msg.velocity.z;
        double dz = spinTop_target_msg.dz;

        position_marker_.action = visualization_msgs::msg::Marker::ADD;
        position_marker_.pose.position.x = xc;
        position_marker_.pose.position.y = yc;
        position_marker_.pose.position.z = za + dz / 2;
        
        armors_marker_.action = visualization_msgs::msg::Marker::ADD;
        armors_marker_.points.clear();

        // Draw armors
        bool is_current_pair = true;
        size_t a_n = spinTop_target_msg.armors_count;
        geometry_msgs::msg::Point p_a;
        double r = 0;
        for (size_t i = 0; i < a_n; i++) {
            double tmp_yaw = yaw + i * (2 * M_PI / a_n);
            // Only 4 armors has 2 radius and height
            if (a_n == 4) {
                r = is_current_pair ? r1 : r2;
                p_a.z = za + (is_current_pair ? 0 : dz);
                is_current_pair = !is_current_pair;
            } 
            else {
                r = r1;
                p_a.z = za;
            }
            p_a.x = xc - r * cos(tmp_yaw);
            p_a.y = yc - r * sin(tmp_yaw);
            armors_marker_.points.emplace_back(p_a);
        }
    } 
    else {
        position_marker_.action = visualization_msgs::msg::Marker::DELETE;
        armors_marker_.action = visualization_msgs::msg::Marker::DELETE;
    }
    
    visualization_msgs::msg::MarkerArray marker_array;

    marker_array.markers.emplace_back(position_marker_);
    marker_array.markers.emplace_back(armors_marker_);
    marker_pub_->publish(marker_array);
}


} // namespace vision_tracker
} // namespace rcia


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rcia::vision_tracker::VisionTrackerNode)