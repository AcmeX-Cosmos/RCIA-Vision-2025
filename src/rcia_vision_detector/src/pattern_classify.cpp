#include "pattern_classify.hpp"

#include "armor_detector.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

PatternClassifier::PatternClassifier() {
    const auto model_path = ament_index_cpp::get_package_share_directory("rcia_vision_detector")
        + "/classify_model/lenet.onnx";
    net = cv::dnn::readNetFromONNX(model_path);
}

// ��ȡ��ROI
void PatternClassifier::getSimpleRoiPattern(const vector<Point> &Draw_Box, const Mat &img, string &armor_type, int &find_count, int& armorPatternIdx, double& armorPatternAcc) {
    // cv::Point2f p0(Draw_Box[0].x + plate_w * 0.125, Draw_Box[0].y - plate_h * 0.75);
    // cv::Point2f p1(Draw_Box[1].x - plate_w * 0.125, Draw_Box[1].y - plate_h * 0.75);
    // cv::Point2f p2(Draw_Box[2].x - plate_w * 0.125, Draw_Box[2].y + plate_h * 0.75);
    // cv::Point2f p3(Draw_Box[3].x + plate_w * 0.125, Draw_Box[3].y + plate_h * 0.75);

    // circle(img, Draw_Box[0], 2, Scalar(255, 0, 255), -1);             
    // circle(img, Draw_Box[1], 2, Scalar(0, 255, 0), -1);                 
    // circle(img, Draw_Box[2], 2, Scalar(255, 255, 0), -1);               
    // circle(img, Draw_Box[3], 2, Scalar(0, 255, 255), -1);                 

    try {
        // 定义灯光长度（图像中的像素长度）
        static const int light_length = 15;
        // 定义透视变换后的图像高度
        static const int warp_height = 28;
        // 定义小型装甲板的宽度
        static const int small_armor_width = 30;
        // 定义大型装甲板的宽度
        static const int large_armor_width = 54;
        // 定义数字区域的大小
        static const cv::Size roi_size(20, 28);
        // 定义输入神经网络的图像大小
        static const cv::Size input_size(28, 28);

        // 定义装甲板上四个灯光点的坐标
        cv::Point2f lights_vertices[4] = {
            Draw_Box[3], Draw_Box[0], Draw_Box[1], Draw_Box[2]};
      

        // 计算顶部灯光在变换后图像中的 y 坐标
        const int top_light_y = (warp_height - light_length) / 2 - 1;
        // 计算底部灯光在变换后图像中的 y 坐标
        const int bottom_light_y = top_light_y + light_length;
        // 根据装甲板类型确定变换后的图像宽度
        const int warp_width = armor_type == "small_armor" ? small_armor_width : large_armor_width;
        // 定义变换后图像的目标坐标点
        cv::Point2f target_vertices[4] = {
            cv::Point(0, bottom_light_y),
            cv::Point(0, top_light_y),
            cv::Point(warp_width - 1, top_light_y),
            cv::Point(warp_width - 1, bottom_light_y),
        };
        // 定义用于存储提取的数字图像的变量
        cv::Mat number_image;
        // 计算透视变换矩阵
        auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
        // 对源图像进行透视变换，得到包含数字的图像
        cv::warpPerspective(img, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

        // 从变换后的图像中提取数字区域
        number_image = number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

        // 将图像转换为灰度图像
        cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
        // 对图像进行二值化处理
        cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        cv::resize(number_image, number_image, input_size);

        //number_image 
        // imshow("number_image", number_image);

        // Mat PatternRoi = getPatternRoi(number_image);
        
        // imshow("PatternRoi", PatternRoi);

        getPatternClassify(number_image, armorPatternIdx, armorPatternAcc, find_count);

    } catch (exception &e) {
        cout << "Exception in getSimpleRoiPattern: error" << endl; // << e.what() 
        armorPatternIdx = 0;
        armorPatternAcc = 0;
    }
}

Mat PatternClassifier::getPatternRoi(const Mat &Simple_RoiThresh) {
    try {
        int roi_h = Simple_RoiThresh.rows;
        int roi_w = Simple_RoiThresh.cols;
        int cx = roi_w / 2;
        int cy = roi_h / 2;

        int Start_x = cx - roi_w * 0.5;
        int Start_y = cy - roi_h * 0.5;
        int End_x = cx + roi_w * 0.5;
        int End_y = cy + roi_h * 0.5;

        Mat Roi_OUT  = Simple_RoiThresh(Rect(Start_x, Start_y, End_x - Start_x, End_y - Start_y));


        cvtColor(Roi_OUT, Roi_OUT, COLOR_BGR2GRAY);

        threshold(Roi_OUT, Roi_OUT, 0, 255, THRESH_BINARY + THRESH_OTSU);

        // cv::imshow("Blur_OUT", Roi_OUT);

        medianBlur(Roi_OUT, Roi_OUT, 3);

        Mat kernel = getStructuringElement(MORPH_RECT, Size(1, 1));

        morphologyEx(Roi_OUT, Roi_OUT, MORPH_OPEN, kernel);

        return Roi_OUT;
        
    } catch (exception &e) {
        cout << "Exception in getPatternRoi: error" << endl; // << e.what()
        return Mat();
    }
}


void PatternClassifier::getPatternClassify(const Mat &PatternRoi, int &armorPatternIdx, double &armorPatternAcc, int &find_count) {

    // cv::imshow("PatternRoi", PatternRoi);

    Mat resized, inputBlob;
    // resize(PatternRoi, resized, Size(20, 28));

    resized = PatternRoi / 255.0;

    dnn::blobFromImage(resized, inputBlob);

    net.setInput(inputBlob);

    cv::Mat output = net.forward();
    
    // 使用 OpenCV 优化方式获取置信度和类别
    cv::Mat outputs = output.reshape(1, 1);
    double confidence;
    cv::Point class_id_point;
    cv::minMaxLoc(outputs, nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    armorPatternIdx = label_id + 1;
    armorPatternAcc = confidence;
}
