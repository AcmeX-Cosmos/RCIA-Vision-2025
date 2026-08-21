// #include "image_subscriber.hpp"
// #include "armor_identify.hpp"

// ImageSubscriber::ImageSubscriber(const rclcpp::NodeOptions &options)
//     : Node("image_subscriber", options), // 传递options给基类
//       last_time_(std::chrono::high_resolution_clock::now()),
//       frame_count_(0), fps_(0.0),
//       current_read_buffer_(&buffer1_), // 初始指向buffer1
//       buffer1_{}, buffer2_{}
// {   
//     // 订阅图像话题
//     img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//         "image_raw", rclcpp::SensorDataQoS(),
//         std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1));
//     RCLCPP_INFO(get_logger(), "Subscribed to image_raw topic");

//     // 订阅
//     serial_sub_ = create_subscription<rcia_vision_interfaces::msg::SerialReceiveData>(
//         "electrl_data",
//         rclcpp::SensorDataQoS().keep_last(1).best_effort(),
//         [this](const rcia_vision_interfaces::msg::SerialReceiveData::SharedPtr msg)
//         {
//             // 获取非当前读的缓冲区进行写入
//             auto *write_buffer =
//                 (current_read_buffer_.load() == &buffer1_) ? &buffer2_ : &buffer1_;

//             // 更新缓冲区数据
//             write_buffer->pitch_angle = msg->pitch_angle;
//             write_buffer->yaw_angle = msg->yaw_angle;
//             write_buffer->bullet_speed = msg->bullet_speed;

//             std::string mecolor = "blue";
//             write_buffer->enemy_color = (mecolor == "blue") ? "red" : "blue";

//             // 原子切换读缓冲区指针
//             current_read_buffer_.store(write_buffer);
//         });
// }

// void ImageSubscriber::init_identifier()
// {
//     is_init_identifier = true;

//     armor_identifier_ = std::make_unique<rcia::vision_identify::ArmorIdentifier>(shared_from_this());
// }

// void ImageSubscriber::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
// {
//     try
//     {
//         if (!is_init_identifier) {
//             init_identifier();
//         }

//         auto src_image = cv_bridge::toCvShare(img_msg, "bgr8")->image;

//         // 检查Mat是否有效
//         if (src_image.empty())
//         {
//             RCLCPP_ERROR(get_logger(), "OpenCV Mat为空，数据解析失败");
//             return;
//         }
//         // 计算帧率
//         auto now = std::chrono::high_resolution_clock::now();
//         frame_count_++;
//         auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time_).count();
//         // 每秒更新一次
//         if (duration >= 1000) { 
//             fps_ = frame_count_ * 1000.0 / duration;
//             frame_count_ = 0;
//             last_time_ = now;
//             // RCLCPP_INFO(get_logger(), "Current FPS: %.2f", fps_);
//             // std::cout << "Current FPS: " << fps_ << std::endl;
//         }

//         auto *read_buffer = current_read_buffer_.load(std::memory_order_acquire);
//         read_buffer->enemy_color = "red";
//         read_buffer->fps = fps_;

//         // 主识别程序
//         armor_identifier_->identify_armor(src_image,read_buffer);

//     }
//     catch (const cv::Exception &e)
//     {
//         RCLCPP_ERROR(get_logger(), "OpenCV异常: %s", e.what());
//     }
// }


// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(ImageSubscriber)