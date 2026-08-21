#ifndef ARMOR_DETECTOR_HPP
#define ARMOR_DETECTOR_HPP


#include "vision_initialize.hpp"

#include "Types.hpp"

// Macro Definition function
#define mymax(a, b) ((a) > (b) ? (a) : (b))
#define mymin(a, b) ((a) < (b) ? (a) : (b))
#define myabs(x) ((x) < 0 ? -(x) : (x))


class ArmorDetector {
public:

    struct DetectorParams
    {
        // 颜色相关
        string enemy_color;
        int red_thresh;
        int blue_thresh;
        
        //几何约束
        double tolerance; // 设置容忍度检查比例
        double max_height_ratio;
        double min_armor_ratio;
        double max_armor_ratio;
        double armor_type_thresh;
        double angle_diff_thresh;

        // 过滤轮廓
        double min_contour_area;
        double area_valid_ratio;
        double max_aspect_ratio;
        int max_contours;
    };

    ArmorDetector(const DetectorParams &dp);

    void Pattern_contrast(Mat& roi_out);

    void compute_armor_info(
        struct rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr,
        rcia_vision_interfaces::msg::ArmorIdentifyInfo& armor_identify_msg);

    void detect_armor_lights(
        Mat &img,
        string enemy_color,
        vector<pair<vector<Point>, string>> *pTrue_list,
        Mat &lights);

    DetectorParams detector_params;

private:
    cv::Mat lut;
    cv::Mat lut2; 

    double gamma = 0.2;
    double gamma2 = 0.5;

    cv::Mat adjusted;
    cv::Mat adjusted2;

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
    cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    
    void color_split(const Mat& img, const string enemy_color, Mat& out_img);

    auto determine_parallelogram(
        const vector<Point2f> &first_BoxPoints,
        const vector<Point2f> &second_BoxPoints);

    pair<Point3f, Point3f> determine_minRect(
        const vector<Point> &first_contour,
        const vector<Point> &second_contour,
        string ratio_status,
        string *pArmor_Type);

    size_t repeated_light_judgment(
        const pair<Point3f, Point3f> &Compare_light_info,
        const vector<pair<Point3f, Point3f>> &compare_point_vector);

    void match_light(vector<pair<vector<Point>, vector<Point2f>>>& LightInfos,
        vector<pair<vector<Point>, string>>* pTrue_list,
        const Mat& gray_image);

    };

#endif