#ifndef PNP_SOLVER_HPP
#define PNP_SOLVER_HPP

#include "Types.hpp"

struct camera_info_struct {
    bool init_flag = false;
    
    string gimbal_name;

    Mat camera_matrix;
    
    Mat dist_coeffs;
};


// PnP
void solve_pnp(
    const struct rcia::vision_identify::GimbalInfoStruct::CameraInfoStruct& camera_info,
    Mat& armor_rvec,
    Mat& armor_tvec,
    rcia_vision_interfaces::msg::ArmorIdentifyInfo& armor_identify_msg
);


void angle_solver(
    rcia_vision_interfaces::msg::GimbalCurrentInfo& gimbal_current_msg,
    rcia_vision_interfaces::msg::ArmorIdentifyInfo& armor_identify_msg
);


void point_solver(
    pair<double, double> angle_degree,
    const struct rcia::vision_identify::GimbalInfoStruct::CameraInfoStruct& camera_info,
    struct rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr
);

template <typename T>
inline T radiansToDegrees(T radians) { 
    return radians * (180.0 / M_PI); 
} 

template <typename T>
inline T degreesToRadians(T degrees) {
    return degrees * (CV_PI / 180.0);
}


#endif