#pragma once
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include "IMVApi.h"
#include <atomic>
#include <chrono>

namespace rcia::camera_driver
{
    class CameraNode : public rclcpp::Node
    {
    public:
        explicit CameraNode(const rclcpp::NodeOptions &options);
        ~CameraNode();

    private:
        // 相机信息管理器和参数
        std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
        // sensor_msgs::msg::CameraInfo::SharedPtr camera_info_; // 修改为智能指针
        sensor_msgs::msg::Image image_msg_;
        sensor_msgs::msg::CameraInfo camera_info_;
        std::string camera_info_url_;
        std::string frame_id_;
        std::string pixel_format_;
        std::string camera_name_;
        int resolution_width_;
        int resolution_height_;

        // SDK相关
        IMV_HANDLE dev_handle_ = nullptr;
        IMV_DeviceList device_list_;
        bool is_grabbing_ = false;

        // ROS组件
        image_transport::CameraPublisher pub_;
        rclcpp::TimerBase::SharedPtr reconnect_timer_;

        // 消息复用
        // sensor_msgs::msg::Image::SharedPtr image_msg_;

        // 性能测量
        std::chrono::steady_clock::time_point last_frame_time_;

        // 初始化方法
        void init_parameters();
        bool init_camera();
        void cleanup_camera();
        void frame_callback(IMV_Frame *frame);
        void try_reconnect();

        // 静态回调包装器
        static void sdk_frame_callback(IMV_Frame *frame, void *user_data)
        {
            static_cast<CameraNode *>(user_data)->frame_callback(frame);
        }
    };
}