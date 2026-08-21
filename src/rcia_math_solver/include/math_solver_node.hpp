// math_solver_node.hpp

#ifndef MATH_SOLVER_NODE_HPP
#define MATH_SOLVER_NODE_HPP

#include "Type.hpp"
#include "ba_solver.hpp"
#include <memory>


namespace rcia{
namespace math_solver{

class MathSolverNode : public rclcpp::Node {
public:
    explicit MathSolverNode(const rclcpp::NodeOptions & options);
    
protected:

    void solver_callback(rcia_vision_interfaces::msg::ArmorIdentifyInfo::SharedPtr armor_identify_msg);


private:
    // BA Solver
    std::unique_ptr <BA_CLASS> ba_solver_;                    // BA 对象声明
    
    rclcpp::Subscription<rcia_vision_interfaces::msg::ArmorIdentifyInfo>::SharedPtr armor_identify_info_sub_;
    rclcpp::Publisher<rcia_vision_interfaces::msg::TargetSpinTop>::SharedPtr spin_top_info_pub_;
};


}
}


#endif // MATH_SOLVER_NODE_HPP