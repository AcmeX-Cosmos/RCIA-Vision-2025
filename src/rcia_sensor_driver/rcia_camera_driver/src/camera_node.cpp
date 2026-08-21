#include "camera_node.hpp"
#include <memory>
#include <cstring>

namespace rcia::camera_driver
{
    CameraNode::CameraNode(const rclcpp::NodeOptions &options)
        : Node("camera_node", options)
    {
        // 参数初始化
        init_parameters();
        
        image_msg_.header.frame_id = frame_id_;
        
        image_msg_.encoding = pixel_format_;
        image_msg_.height = resolution_height_;
        image_msg_.width = resolution_width_;
        image_msg_.step = resolution_width_ * 3; // 假设 BGR8 格式
        image_msg_.data.resize(image_msg_.height * image_msg_.step);

        // 初始化相机信息管理器
        camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "camera");

        camera_info_.width = image_msg_.width;
        camera_info_.height = image_msg_.height;
        camera_info_.header.frame_id = frame_id_;
        camera_info_.header.stamp = this->now();
        // 创建图像发布器

        auto qos = rmw_qos_profile_sensor_data;
        pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

        // 首次尝试初始化相机
        if (!init_camera())
        {
            RCLCPP_ERROR(get_logger(), "Initial camera initialization failed. Retrying...");
            // 设置重连定时器（每2秒重试一次）
            reconnect_timer_ = this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(&CameraNode::try_reconnect, this));
        }
    }

    CameraNode::~CameraNode()
    {
        cleanup_camera();
    }

    void CameraNode::init_parameters()
    {
        // camera_name_ = this->declare_parameter("camera_name", "huarry");
        frame_id_ = this->declare_parameter("camera_frame_id", "camera_optical_frame");
        pixel_format_ = this->declare_parameter("pixel_format", "bgr8");
        resolution_width_ = this->declare_parameter("resolution_width", 1280);
        resolution_height_ = this->declare_parameter("resolution_height", 1024);
        // RCLCPP_INFO(get_logger(), "Resolution: %dx%d", resolution_width_, resolution_height_);
    }

    bool CameraNode::init_camera()
    {
        // 清理之前的资源
        cleanup_camera();

        // 枚举设备
        if (IMV_EnumDevices(&device_list_, interfaceTypeAll) != IMV_OK)
        {
            RCLCPP_ERROR(get_logger(), "No devices found!");
            return false;
        }

        // 创建设备句柄
        unsigned int index = 0;
        if (IMV_CreateHandle(&dev_handle_, modeByIndex, &index) != IMV_OK)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create device handle");
            return false;
        }

        // 打开设备
        if (IMV_Open(dev_handle_) != IMV_OK)
        {
            RCLCPP_ERROR(get_logger(), "Failed to open device");
            return false;
        }

        // 配置采集参数
        IMV_SetBufferCount(dev_handle_, 10);

        // 注册回调函数
        if (IMV_AttachGrabbing(dev_handle_, &CameraNode::sdk_frame_callback, this) != IMV_OK)
        {
            RCLCPP_ERROR(get_logger(), "Failed to register callback");
            return false;
        }

        // 启动采集
        uint64_t maxImagesGrabbed = 0;
        IMV_EGrabStrategy strategy = grabStrartegyLatestImage;
        if (IMV_StartGrabbingEx(dev_handle_, maxImagesGrabbed, strategy) != IMV_OK)
        {
            RCLCPP_ERROR(get_logger(), "Failed to start grabbing");
            return false;
        }

        is_grabbing_ = true;
        return true;
    }

    void CameraNode::cleanup_camera()
    {
        if (dev_handle_)
        {
            if (is_grabbing_)
            {
                IMV_StopGrabbing(dev_handle_);
                is_grabbing_ = false;
            }
            IMV_Close(dev_handle_);
            IMV_DestroyHandle(dev_handle_);
            dev_handle_ = nullptr;
        }
    }

    void CameraNode::try_reconnect()
    {
        RCLCPP_INFO(get_logger(), "Attempting to reconnect to camera...");
        if (init_camera())
        {
            RCLCPP_INFO(get_logger(), "Camera reconnected successfully!");
            reconnect_timer_->cancel(); // 停止重连定时器
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Reconnection failed. Retrying in 2 seconds...");
        }
    }
    void CameraNode::frame_callback(IMV_Frame *frame)
    {
        if (frame->pData == nullptr || frame->frameInfo.size <= 0)
        {
            std::cout << "Invalid frame data or status" << std::endl;
            return;
        }

        if (frame->frameInfo.width <= 0 || frame->frameInfo.height <= 0)
        {
            RCLCPP_ERROR(get_logger(), "Invalid frame dimensions: %d x %d", frame->frameInfo.width, frame->frameInfo.height);
            return;
        }

        if (frame->frameInfo.status == IMV_OK)
        {
            

            IMV_PixelConvertParam stPixelConvertParam;
            static std::vector<unsigned char> convertBuffer;
            // std::cout << "frame->frameInfo.width:" << frame->frameInfo.width << "  frame->frameInfo.height:" << frame->frameInfo.height << std::endl;
            // size_t bufferSize = static_cast<size_t>(frame->frameInfo.width) * frame->frameInfo.height * 3;
            size_t bufferSize = (frame->frameInfo.width + frame->frameInfo.paddingX) * frame->frameInfo.height * 3;
            // std::cout << "bufferSize:" << bufferSize << std::endl;

            if (bufferSize > std::numeric_limits<size_t>::max() / 3)
            { // 防止溢出
                RCLCPP_ERROR(get_logger(), "Buffer size overflow: %zu", bufferSize);
                return;
            }
            convertBuffer.resize(bufferSize);


            // 设置像素转换参数
            stPixelConvertParam.nWidth = frame->frameInfo.width;
            stPixelConvertParam.nHeight = frame->frameInfo.height;
            stPixelConvertParam.ePixelFormat = frame->frameInfo.pixelFormat;
            stPixelConvertParam.pSrcData = frame->pData;
            stPixelConvertParam.nSrcDataLen = frame->frameInfo.size;
            stPixelConvertParam.nPaddingX = frame->frameInfo.paddingX;
            stPixelConvertParam.nPaddingY = frame->frameInfo.paddingY;
            stPixelConvertParam.eBayerDemosaic = demosaicBilinear; // 双线性插值去马赛克
            stPixelConvertParam.eDstPixelFormat = gvspPixelBGR8;   // 转换目标格式为BGR8
            stPixelConvertParam.pDstBuf = convertBuffer.data();
            stPixelConvertParam.nDstBufSize = convertBuffer.size();
            // image_msg_.step = stPixelConvertParam.nWidth * 3 + stPixelConvertParam.nPaddingX;

            // 执行像素格式转换
            if (IMV_PixelConvert(dev_handle_, &stPixelConvertParam) != IMV_OK)
            {
                std::cerr << "Image conversion failed. " << std::endl;
                return;
            }
            image_msg_.header.stamp = camera_info_.header.stamp = this->now();
            image_msg_.data.assign(convertBuffer.begin(), convertBuffer.end());

            // 发布消息
            pub_.publish(image_msg_, camera_info_);
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rcia::camera_driver::CameraNode)