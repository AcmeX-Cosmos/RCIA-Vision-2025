#ifndef BA_SOLVER_HPP
#define BA_SOLVER_HPP

#include "Type.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <memory>

class BA_CLASS{
public:

	explicit BA_CLASS(std::weak_ptr<rclcpp::Node> node);
	~BA_CLASS() = default;

	struct Ba_Param {
		double trans_vector_optimization;
		double yaw_optimization;
		double maximum_yaw;
		double minimum_yaw;
	};

	Ba_Param ba_param_;

	std::weak_ptr<rclcpp::Node> node_;

	ceres::Solver::Options options;
	ceres::Solver::Options options2;

    void Solver(rcia_vision_interfaces::msg::ArmorIdentifyInfo::SharedPtr armor_identify_msg);


protected:

	void init_camra_info();

	void adjust_optimized_data(vector<Point2f>& object_2d_point);

	void bundle_adjustment(
		vector<Mat>& extrinsics,
		vector<vector<Point2f>>& image_points,
		const uint8_t& armor_pattern_idx);
		
	void calculate_armor_eulerAngles();

	void publish_armor_bapose(rcia_vision_interfaces::msg::ArmorIdentifyInfo::SharedPtr armor_identify_msg);

	void publish_armor_marker();

	vector<Mat> createExtrinsicsMatrix(const vector<Mat>& rotations, const vector<Mat>& translations);

private:

	static const int optimizeLength_ = 5;

	static constexpr double TARGET_PITCH_DAG = 15.0;

	vector<Mat> rotations;		
	vector<Mat> translations;	
	
	vector<vector<Point2f>> image_points;

	rcia_vision_interfaces::msg::ArmorBaposeInfo armor_bapose_msg_;

	rclcpp::Publisher<rcia_vision_interfaces::msg::ArmorBaposeInfo>::SharedPtr armor_bapose_pub_;

	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr armor_marker_pub_;
};



#endif
