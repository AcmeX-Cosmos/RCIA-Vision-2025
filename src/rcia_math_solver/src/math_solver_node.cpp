// math_solver_node.cpp

#include "math_solver_node.hpp"

namespace rcia{
namespace math_solver{

MathSolverNode::MathSolverNode(const rclcpp::NodeOptions &options) :
    Node("math_solver_node", options),
    ba_solver_(nullptr)
{   
	// 创建订阅者
	armor_identify_info_sub_ = this->create_subscription<rcia_vision_interfaces::msg::ArmorIdentifyInfo>(
        "armor_identify_info", 2, [this](const rcia_vision_interfaces::msg::ArmorIdentifyInfo::SharedPtr armor_identify_msg) {
            solver_callback(armor_identify_msg);
        });

	// 创建发布者
	spin_top_info_pub_ = this->create_publisher<rcia_vision_interfaces::msg::TargetSpinTop>("spin_top_topic", 2);
};


void MathSolverNode::solver_callback(rcia_vision_interfaces::msg::ArmorIdentifyInfo::SharedPtr armor_identify_msg) {
        if (ba_solver_ == nullptr) {
            ba_solver_ = std::make_unique<BA_CLASS>(weak_from_this());
        }

        ba_solver_->Solver(armor_identify_msg);
    }
} // namespace math_solver
} // namespace rcia

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rcia::math_solver::MathSolverNode)