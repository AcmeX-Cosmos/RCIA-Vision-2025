
#include "armor_detector.hpp"
#include "corner_correct.hpp"


/**
 * @details
 * 1. identify_armor: 识别装甲板特征，获取初步的灯条信息。
 * 2. detect_armor_lights: 获取所有可能的灯条信息，准备进行匹配。
 * 3. color_split: 对获取到的图像进行颜色分离，二值化处理。
 * 4. paralleGetLightInfo: 对获取到的灯条信息进行并行处理，提高效率。
 * 5. match_light: 将轮廓进行两两匹配，计算所构成的向量比例。
 * 6. determine_parallelogram: 设定容忍值比例，判断是否构成平行。
 * 7. determine_minRect: 确定最小外接矩形，进一步精确定位。
 * 8. repeated_light_judgment: 进行重复灯条判断，剔除无效数据。
 */
// +-----------------------------------------------------------------------+
// |                                                                       |
// |              identify_armor -> detect_armor_lights                    |
// |                 detect_armor_lights -> color_split                    |
// |                 color_split -> paralleGetLightInfo                    |
// |              paralleGetLightInfo -> match_light                       |
// |                                                                       |
// +-----------------------------------------------------------------------+

// +------------------------------------------------------------------------+
// |                                                                        |
// |                   match_light -> determine_parallelogram               |
// |          determine_parallelogram -> determine_minRect                  |
// |             determine_minRect -> repeated_light_judgment               |
// |                                                                        |
// +------------------------------------------------------------------------+




// # --------------------------------------------------------------------- #
// # --------------------- # ② 通过颜色分离，二值化处理 # ------------------- # 
// # ------------ # 将图像中的"敌方颜色 " 部分从整个图像中提取出来 # ------------ # 
// # --------------------------------------------------------------------- #
// 增加对比度：伽玛校正
ArmorDetector::ArmorDetector(const DetectorParams &dp) : detector_params(dp)
{
    // 初始化LUT
    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for(int i = 0; i < 256; ++i) {
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, 1.0 / gamma) * 255.0);
    }
    lut = lookUpTable;

    cv::Mat lookUpTable2(1, 256, CV_8U);
    uchar* p2 = lookUpTable.ptr();
    for(int i = 0; i < 256; ++i) {
        p2[i] = cv::saturate_cast<uchar>(pow(i / 255.0, 1.0 / gamma2) * 255.0);
    }
    lut2 = lookUpTable2;
}

void ArmorDetector::color_split(const Mat &img, const string enemy_color, Mat &out_img)
{
    // 应用查找表进行伽玛校正
    // cv::LUT(img, lut, adjusted);
    vector<Mat> channels;

    split(img, channels);

    Mat B = channels[0];
    // Mat G = channels[1];
    Mat R = channels[2];

    double maxval1 = 255; // optimization is needed here
    if (detector_params.enemy_color == "red") {
        Mat R_thresh;
        subtract(R, B, R_thresh);
        threshold(R_thresh, out_img, detector_params.red_thresh, maxval1, THRESH_BINARY);
        // cv::dilate(out_img, out_img, kernel);
        cv::erode(out_img, out_img, kernel);
    }
    else if (detector_params.enemy_color == "blue") {
        Mat B_thresh;
        subtract(B, R, B_thresh);
        threshold(B_thresh, out_img, detector_params.blue_thresh, maxval1, THRESH_BINARY);
        cv::dilate(out_img, out_img, kernel2);
        // cv::erode(out_img, out_img, kernel);
    }   
    
    // cv::imshow("out_img", out_img);
}

void ArmorDetector::Pattern_contrast(Mat &roi_out) {
    cv::LUT(roi_out, lut2, roi_out);
}




// # ----------------------------------------------------------------------------- #
// # -------------------- # ⑥ 确定两个矩形框是否构成平行四边形 # --------------------- #
// # ----------- # 计算两个矩形框的四个点，判断它们是否能够构成一个平行四边形 # ----------- #
// # ---------------------------------------------------------------------------- #
auto ArmorDetector::determine_parallelogram(const vector<Point2f> &first_BoxPoints, const vector<Point2f> &second_BoxPoints)
{
    // 定义装甲板的四个点
    Point LT((first_BoxPoints[0] + first_BoxPoints[1]) / 2.0);                      //LeftTop_point
    Point RT((second_BoxPoints[0] + second_BoxPoints[1]) / 2.0);                     //RightTop_point
    Point RB((second_BoxPoints[2] + second_BoxPoints[3]) / 2.0);                     //RightBottom_point
    Point LB((first_BoxPoints[2] + first_BoxPoints[3]) / 2.0);                      //LeftBottom_point

    // 计算是否构成平行四边形的向量
    Point2f Vector_LTRT = RT - LT;
    Point2f Vector_RTRB = RB - RT;
    Point2f Vector_RBLB = LB - RB;
    Point2f Vector_LBLT = LT - LB;

    // 检查向量比例
    float ratio1 = (Vector_RBLB.x != 0) ? Vector_LTRT.x / Vector_RBLB.x : numeric_limits<float>::infinity();
    float ratio2 = (Vector_LBLT.x != 0) ? Vector_RTRB.x / Vector_LBLT.x : numeric_limits<float>::infinity();

    // 设置容忍度检查比例
    // float tolerance = 1.25;

    /*cout << "ratio1: " << ratio1 << "\nratio2: " << ratio2 << "\n\n" << endl;*/
    // cout << abs(ratio1 - ratio2) << endl;
    if (abs(ratio1 - ratio2) < detector_params.tolerance)
    {
        // 返回平行四边形的四个点 和 "True"
        return make_pair(vector<Point>{LT, RT, RB, LB}, "True");
    }
    else if (isinf(ratio1) || isinf(ratio2)) {
        // 返回平行四边形的四个点 和 "Error ratio"
        return make_pair(vector<Point>{LT, RT, RB, LB}, "Error ratio");
    }
    else {
        // 返回空对象 和 "False"
        return make_pair(vector<Point>{}, "False");
    }
}

// # ----------------------------------------------------------------- #
// # ---------------- # ⑦ 将平行灯条轮廓计算最小外接矩形 # --------------- # 
// # ----------- # 利用灯条是否构成装甲板的几何特征进行二次筛选 # ----------- # 
// # -------- # 既利用灯条中心点的高度差、灯条长宽比和灯条角度差进行判断 # ---- # 
// # ----------------------------------------------------------------- #
pair<Point3f, Point3f> ArmorDetector::determine_minRect(const vector<Point> &first_contour, const vector<Point> &second_contour, string ratio_status, string *pArmor_Type)
{
    try {
        bool Debugging = 0;

        RotatedRect first_minRect = minAreaRect(first_contour);         //左边灯条最小外接矩形
        RotatedRect second_minRect = minAreaRect(second_contour);       //右边灯条最小外接矩形

        // Fx, Fy, Fw, Fh 左边灯条的中心点坐标与尺寸
        int Fx = first_minRect.center.x, Fy = first_minRect.center.y,
            Fw = first_minRect.size.width, Fh = first_minRect.size.height;

        // Sx, Sy, Sw, Sh 右边灯条的中心点坐标与尺寸
        int Sx = second_minRect.center.x, Sy = second_minRect.center.y,
            Sw = second_minRect.size.width, Sh = second_minRect.size.height;

        // minAreaRect 宽高标准化处理
        Fw > Fh ? swap(Fw, Fh) : void();
        Sw > Sh ? swap(Sw, Sh) : void();

        float Fa = fitEllipse(first_contour).angle;        //左边灯条的角度
        float Sa = fitEllipse(second_contour).angle;       //右边灯条的角度

        Point3f first_info = Point3f(Fx, Fy, Fa);
        Point3f second_info = Point3f(Sx, Sy, Sa);

        // cout << "----------------------" << endl;
        // cout << "Fy" << Fy << "Sy" << Sy << endl;
        // cout << "Fh" << Fh << "Sh" << Sh << endl;
        // cout << abs(Fy - Sy) << "    " << min(Fh, Sh) << endl;
        // cout << "----------------------\n\n" << endl;

        // // 两个灯条的Y之间差距 + 两个高度差距 与最小高度做比较
        // // 两个相对高度距离与最小高度 做比较
        // if ((myabs(Fh - Sh) + myabs(Fy - Sy) > mymin(Fh, Sh) * 2)) {
        //     if (Debugging)  {cout << "Identify error as Relative Y distance difference 1" << endl;}
        //     // cout << myabs(Fh - Sh)  << "     "  <<  mymin(Fy, Sy) << endl;
        //     return {{0, 0, 0}, {0, 0, 0}};
        // }

        // // 两个相对高度差距与两个灯条的Y之间差距 做比较
        // if (myabs(Fh - Sh) * 1.5 >= myabs(Fy - Sy)) {
        //     if (Debugging)  {cout << "Identify error as Relative Y distance difference 1" << endl;}
        //     // cout << myabs(Fh - Sh)  << "     "  <<  mymin(Fy, Sy) << endl;
        //     return {{0, 0, 0}, {0, 0, 0}};
        // }

        // 两个灯条的Y之间差距 不大于最小高度
        // 两个相对高度距离与最小高度 做比较
        if (myabs(Fy - Sy) > mymin(Fh, Sh) * 2) {
            if (Debugging)  {cout << "Identify error as Relative Y distance difference 2" << endl;}
            // cout << myabs(Fy - Sy)  << "     "  <<  mymin(Fh, Sh) << endl;
            return {{0, 0, 0}, {0, 0, 0}};
        }

        if (mymax(Fh, Sh) / mymin(Fh, Sh) >= detector_params.max_height_ratio)
        {
            if (Debugging)  {cout << "Identify error as Comparing heights" << endl;}
            return {{0, 0, 0}, {0, 0, 0}};
        }

        // 两个灯条的之间的X距离 比较最小高度
        // 两点之间距离与最小高度 比较
        // 大装甲板与小装甲板阈值不相同 需要调整
        float TwoPoints_distance = sqrt(pow((Fx - Sx), 2) + pow((Fy - Sy), 2));
        float Armor_aspectratio = TwoPoints_distance / min(Fh, Sh);

        /*cout << TwoPoints_distance << "\t\t" << min(Fh, Sh) << "\t\t" << Armor_aspectratio << endl;*/
        /*cout << "Armor_aspectratio: " << Armor_aspectratio << endl;*/
        if (Armor_aspectratio < detector_params.min_armor_ratio || Armor_aspectratio > detector_params.max_armor_ratio)
        {
            if (Debugging)  {cout << "Identify error as TwoPoints distance" << endl;}
            return {{0, 0, 0}, {0, 0, 0}};
        }
        else if (Armor_aspectratio < detector_params.armor_type_thresh)
        {
            *pArmor_Type = "small_armor";
        }
        else {
            *pArmor_Type = "large_armor";
        }

        // 如果平行四边形角度为零 做异常处理 拟用椭圆得到角度 再做比较
        // if (ratio_status == "Error ratio") {}
        if (first_contour.size() < 5 || second_contour.size() < 5) {
            if (Debugging)  {cout << "Identify error as Need 5 point ellipses" << endl;}
            return {{0, 0, 0}, {0, 0, 0}};
        }

        Fa = (Fa > 165) ? Fa - 180 : Fa;
        Sa = (Sa > 165) ? Sa - 180 : Sa;
        float Angle_difference = abs(Fa - Sa);                  //两边灯条角度差
        // cout << Angle_difference << endl;
        if (detector_params.angle_diff_thresh <= Angle_difference)
        { //&& Angle_difference <= 175) {
            if (Debugging)  {cout << "Identify error as Ellipse ratio" << endl;}
            return {{0, 0, 0}, {0, 0, 0}};
        }

        return {first_info, second_info};
    }

    catch (const exception& e) {
        // 捕获异常并进行处理
        // cerr << "Error occurred in determine_minRect function." << e.what() << endl;
        return {{0, 0, 0}, {0, 0, 0}};
    }
}



// # ----------------------------------------------------------------------- #
// # ----- # ⑧ 取三个点与历史重复的灯条做比较, 目的是取更符合装甲板的灯条 # --------- # 
// # ----------------------- # point.x 为 灯条 中心x # ----------------------- # 
// # ----------------------- # point.y 为 灯条 中心y # ----------------------- # 
// # ----------------------- # point.z 为 灯条 角度   # ---------------------- # 
// # ----------------------------------------------------------------------- #
size_t ArmorDetector::repeated_light_judgment(const pair<Point3f, Point3f> &Compare_light_info, const vector<pair<Point3f, Point3f>> &compare_point_vector)
{
    size_t compare_index = 0;
    Point3f compare_target1, compare_target2, compare_point, same_point;

    bool compare_found = false;

    tie(compare_target1, compare_target2) = Compare_light_info;

    
    for (const auto& pair : compare_point_vector) {
        if (pair.first == compare_target1 || pair.first == compare_target2) {
            same_point = pair.first;
            compare_point = pair.second;
            compare_found = true;
            break;
        }
        else if (pair.second == compare_target1 || pair.second == compare_target2) {
            same_point = pair.second;
            compare_point = pair.first;
            compare_found = true;
            break;
        }
        compare_index++;
    }

    if (compare_found) {
        same_point.z = (same_point.z > 165) ? same_point.z - 180 : same_point.z;
        compare_point.z = (compare_point.z > 165) ? compare_point.z - 180 : compare_point.z;
        compare_target1.z = (compare_target1.z > 165) ? compare_target1.z - 180 : compare_target1.z;
        compare_target2.z = (compare_target2.z > 165) ? compare_target2.z - 180 : compare_target2.z;

        float SaD, SyD;
        float FaD = abs(compare_point.z - same_point.z);
        float FyD = abs(compare_point.y - same_point.y);

        if (same_point == compare_target1) {
            SaD = abs(compare_target2.z - same_point.z);
            SyD = abs(compare_target2.y - same_point.y);
        }
        else {
            SaD = abs(compare_target1.z - same_point.z);
            SyD = abs(compare_target2.y - same_point.y);
        }

        // 角度差 和 高度差 权重比较
        if (FaD * 0.5 + FyD * 0.5 > SaD * 0.5 + SyD * 0.5) {
            // cout << compare_target1 << "  " << compare_target2 << "\n\n" << endl;
            return compare_index;
        }
        else{
            return -2;
        }
    }
    
    return -1;
}


// # ----------------------------------------------------------------------- #
// # ------------------------- # ⑤ 两两比较灯条信息  # ----------------------- # 
// # --------------------- # 根据给定的条件判断是否匹配成功 # ------------------- # 
// # ---------------- # 判断新匹配的灯条是否与之前已匹配的灯条重复 # -------------- # 
// # ----------------------- # 挑选出更符合装甲的灯条 # ------------------------ # 
// # -------------------------- # 返回匹配到的结果 # -------------------------- # 
// # ----------------------------------------------------------------------- #
LightCornerCorrector light_corrector;
void ArmorDetector::match_light(vector<pair<vector<Point>, vector<Point2f>>>& LightInfos,
    vector<pair<vector<Point>, string>>* pTrue_list,
    const Mat& gray_image)
{
    String determine_minRect_result, Armor_Type;

    vector<pair<Point3f, Point3f>> compare_point_vector;

    pair<Point3f, Point3f> Compare_light_info;

    // cout << LightInfos.size() << endl;
    // 此处可以优化 由于之前多线程加的 空元素 需要删除空元素
    // 可以优化去除掉空的元素 然后不用在此循环
    for (auto it = LightInfos.rbegin(); it != LightInfos.rend();) {
        if (it->first.empty() || it->second.empty()) {
            LightInfos.erase(next(it).base());
        } else {
            ++it;
        }
    }
    
    sort(LightInfos.begin(), LightInfos.end(), [](const auto& a, const auto& b) {
        return a.first[0].x < b.first[0].x;
    });

    // 时间复杂度高，可优化。
    // 可能可以用二分查找
    for (auto it1 = LightInfos.begin(); it1 != LightInfos.end(); ++it1) {
        vector<Point> first_contour = it1->first;
        vector<Point2f> first_BoxPoints = it1->second;

        for (auto it2 = next(it1); it2 != LightInfos.end(); ++it2) {
            vector<Point> second_contour = it2->first;
            vector<Point2f> second_BoxPoints = it2->second;

            auto [Determine_BoxPoints_result, ratio_status] = determine_parallelogram(first_BoxPoints, second_BoxPoints);

            if (ratio_status != "False") {
                // drawContours(*img_ptr, Determine_BoxPoints_result, 0, Scalar(0, 255, 0), 3);
                Compare_light_info = determine_minRect(first_contour, second_contour, ratio_status, &Armor_Type);

                if (Compare_light_info.first.x != 0 && Compare_light_info.second.x != 0) {
                    if (!compare_point_vector.empty()) {
                        size_t compare_index = repeated_light_judgment(Compare_light_info, compare_point_vector);
                        
                        if (compare_index != -1 && compare_index != -2) {
                            pTrue_list->erase(pTrue_list->begin() + compare_index);
                            compare_point_vector.erase(compare_point_vector.begin() + compare_index);
                        }
                        else if(compare_index == -2){
                            continue;
                        }
                    }
                    // PCA角点矫正
                    // // 已知正确的{LT, RT, RB, LB}，first_contour, second_contour, gray_image
                    // auto result = light_corrector.correctCorners(first_contour,second_contour, gray_image);
                    // if(result.second=="True"){
                    //     Determine_BoxPoints_result = result.first;
                    //     // cout<<"corner_correct"<<endl;
                    // }

                    pTrue_list->emplace_back(make_pair(Determine_BoxPoints_result, Armor_Type));
                    compare_point_vector.emplace_back(Compare_light_info);
                }
            }
        }
    }
}

// # ------------------------------------------------------------------------------ #
// # ------------------------- # ③ 并行处理轮廓并提取灯条信息 # ----------------------- #
// # --------------- # 通过 "面积或长宽比不符合等要求" 过滤掉不需要的轮廓 # --------------- #
// # ------------ # 计算有效轮廓的矩形框点 以线程安全的方式添加到灯条信息列表中# ------------ #
// # ------------------------------------------------------------------------------ #
// class paralleGetLightInfo : public ParallelLoopBody {
// public:
//     paralleGetLightInfo(vector<vector<Point>>& _contours,
//                     vector<Vec4i>& _hierarchy,
//                     Mat& _img,
//                     vector<pair<vector<Point>, vector<Point2f>>>& _lightInfoVector)
//         : contours(_contours), hierarchy(_hierarchy), img(_img), lightInfoVector(_lightInfoVector),
//           mtx() { }

//     void operator()(const Range& range) const override {
//         vector<pair<vector<Point>, vector<Point2f>>> localList;

//         for (size_t index = range.start; index < range.end; ++index) {
//             if (hierarchy[index][3] == -1) {
//                 RotatedRect minRect = minAreaRect(contours[index]);
//                 Rect boundRect = boundingRect(contours[index]);
//                 /*cout << boundingRect(contours[index]) << endl;*/

//                 float area = contourArea(contours[index]);
//                 float width = static_cast<float>(minRect.size.width);
//                 float height = static_cast<float>(minRect.size.height);
//                 //fitEllipseNoDirect中应至少有5个点来拟合椭圆, 如果像素点过少会出error
//                 float aspectRatio = static_cast<float>(boundRect.width) / static_cast<float>(boundRect.height);

//                 // cout << "contourArea: \t" << area << endl;
//                 // cout << "aspectRatio: \t" << aspectRatio << endl;
//                 // cout << (area < 10.0 )<< (area < (width * height * 0.5)) << (aspectRatio >= 2.5) << endl;
//                 if (area < 10.0 || area < width * height * 0.5 || aspectRatio >= 2.5) {
//                     continue;
//                 }

//                 // 计算 Box_Points
//                 vector<Point2f> Box_Points(4);
//                 minRect.points(Box_Points.data());

//                 // 计算 row_sums
//                 vector<int> row_sums(4);
//                 for (int i = 0; i < 4; ++i) {
//                     double round_temp = Box_Points[i].x + (Box_Points[i].y) * 1.25;
//                     row_sums[i] = static_cast<int>(round(round_temp));
//                 }

//                 // 对 row_sums 进行排序
//                 vector<int> sorted = {0,1,2,3};    // 填充 sorted 数组为 0, 1, 2, 3
//                 sort(sorted.begin(), sorted.end(), [&row_sums](int a, int b) {
//                     return row_sums[a] < row_sums[b];
//                 });

//                 // 根据排序结果重新排列 Box_Points
//                 vector<Point2f> Sorted_Box_Points;
//                 for (int i = 0; i < 4; ++i) {
//                     Sorted_Box_Points.emplace_back(Box_Points[sorted[i]]);
//                 }
                
//                 if (Sorted_Box_Points[2].x > Sorted_Box_Points[1].x && Sorted_Box_Points[1].y > Sorted_Box_Points[2].y){
//                     // cout << "Sorted_Box_Points[2].x > Sorted_Box_Points[1].x" << (Sorted_Box_Points[2].x > Sorted_Box_Points[1].x) << endl;
//                     // cout << "Sorted_Box_Points[1].y > Sorted_Box_Points[2].y" << (Sorted_Box_Points[1].y > Sorted_Box_Points[2].y) << endl;
//                     continue;
//                 }

//                 // 将灯条的四个点 标上数字
//                 // for (int i = 0; i < 4; ++i) {
//                 //    int offset_x = (i % 2 == 0) ? -12 : 0;
//                 //    putText(img, to_string(i + 1), Point(Sorted_Box_Points[i].x+offset_x, Sorted_Box_Points[i].y),
//                 //        FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 255, 0), 3);
//                 // }

//                 localList.emplace_back(make_pair(contours[index], Sorted_Box_Points));
//             }

//             {
//             unique_lock<mutex> lock(mtx);

//             lightInfoVector.insert(lightInfoVector.end(),
//                                     make_move_iterator(localList.begin()),
//                                     make_move_iterator(localList.end()));
//             }
//         }
//     }

// private:
//     vector<vector<Point>>& contours;
//     vector<Vec4i>& hierarchy;
//     Mat& img;
//     vector<pair<vector<Point>, vector<Point2f>>>& lightInfoVector;
//     mutable mutex mtx;
    
// };

// # ------------------------------------------------------------------------ #
// # ---------------------- #  ① 从输入图像中提取灯条信息  # ------------------- # 
// # ---------- # 返回匹配到的灯条信息、颜色分离后的图像以及原始输入图像 # ----------- # 
// # ------------------------------------------------------------------------ #
void ArmorDetector::detect_armor_lights(Mat &img, string enemy_color, vector<pair<vector<Point>, string>> *pTrue_list, Mat &lights)
{
    vector<pair<vector<Point>, vector<Point2f>>> lightInfoVector;

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat gray_image;
    cvtColor(img, gray_image, COLOR_BGR2GRAY);
    
    try {
        if (enemy_color == "red") {
            color_split(img, enemy_color, lights);
        }
        else if (enemy_color == "blue") {
            color_split(img, enemy_color, lights);
        }

        findContours(lights, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 排序并获取前50个最大面积的轮廓
        sort(contours.begin(), contours.end(), [](const vector<cv::Point>& a, const vector<cv::Point>& b) {
            return cv::contourArea(a) > cv::contourArea(b);
        });

        if (contours.size() > detector_params.max_contours)
        {
            contours.resize(detector_params.max_contours);
        }

        // paralleGetLightInfo process(contours, hierarchy, img, lightInfoVector);

        // parallel_for_(Range(0, contours.size()), process);

        for (size_t index = 0; index < contours.size(); index++) {

            RotatedRect minRect = minAreaRect(contours[index]);

            Rect boundRect = boundingRect(contours[index]);
            /*cout << boundingRect(contours[index]) << endl;*/

            float area = contourArea(contours[index]);
            float width = static_cast<float>(minRect.size.width);
            float height = static_cast<float>(minRect.size.height);
            //fitEllipseNoDirect中应至少有5个点来拟合椭圆, 如果像素点过少会出error
            float aspectRatio = static_cast<float>(boundRect.width) / static_cast<float>(boundRect.height);

            // cout << "contourArea: \t" << area << endl;
            // cout << "aspectRatio: \t" << aspectRatio << endl;
            // cout << (area < 10.0 )<< (area < (width * height * 0.5)) << (aspectRatio >= 2.5) << endl;

            // 此处瞄远处要用 area < 15.0
            if (area < detector_params.min_contour_area || area < (width * height * detector_params.area_valid_ratio) || aspectRatio >= detector_params.max_aspect_ratio)
            {
                continue;
            }

            // 计算 Box_Points
            vector<Point2f> Box_Points(4);
            minRect.points(Box_Points.data());

            // 计算 row_sums
            vector<int> row_sums(4);
            for (int i = 0; i < 4; ++i) {
                double round_temp = Box_Points[i].x + (Box_Points[i].y);// * 1.25;
                row_sums[i] = static_cast<int>(round(round_temp));
            }

            // 对 row_sums 进行排序
            vector<int> sorted = {0,1,2,3};    // 填充 sorted 数组为 0, 1, 2, 3
            sort(sorted.begin(), sorted.end(), [&row_sums](int a, int b) {
                return row_sums[a] < row_sums[b];
            });

            // 根据排序结果重新排列 Box_Points
            vector<Point2f> Sorted_Box_Points;
            Sorted_Box_Points.reserve(4);
            for (int i = 0; i < 4; ++i) {
                Sorted_Box_Points.emplace_back(Box_Points[sorted[i]]);
            }
            
            if (Sorted_Box_Points[2].x > Sorted_Box_Points[1].x && Sorted_Box_Points[1].y > Sorted_Box_Points[2].y){
                // cout << "Sorted_Box_Points[2].x > Sorted_Box_Points[1].x" << (Sorted_Box_Points[2].x > Sorted_Box_Points[1].x) << endl;
                // cout << "Sorted_Box_Points[1].y > Sorted_Box_Points[2].y" << (Sorted_Box_Points[1].y > Sorted_Box_Points[2].y) << endl;
                continue;
            }

            // // 将灯条的四个点 标上数字
            // for (int i = 0; i < 4; ++i) {
            //    int offset_x = (i % 2 == 0) ? -12 : 0;
            //    putText(img, to_string(i + 1), Point(Sorted_Box_Points[i].x+offset_x, Sorted_Box_Points[i].y),
            //        FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 255, 0), 3);
            // }

            lightInfoVector.emplace_back(make_pair(contours[index], Sorted_Box_Points));
            
        }

        // resize(lights, lights, Size(), 0.75, 0.75);
        // imshow("lights", lights);

        match_light(lightInfoVector, pTrue_list, gray_image);

    }
    catch (const exception& e) {
        // 捕获异常并进行处理
        cerr << "Error occurred in detect_armor_lights function." << e.what() << endl;
    }
}


/**
 *   compute_armor_info
 * @brief 计算装甲板信息的函数
 * 
 * 该函数接收一个包含装甲板四个顶点的向量，并计算装甲板的宽度、高度和中心坐标。
 * 计算结果会存储在传入的结构体指针中。
 * 
 * @param best_armor_contours_ 包含装甲板四个顶点的向量
 * @param identify_info_ptr 指向存储计算结果的结构体的指针
 */
void ArmorDetector::compute_armor_info(struct rcia::vision_identify::IdentifyInfoStruct *identify_info_ptr, rcia_vision_interfaces::msg::ArmorIdentifyInfo &armor_identify_msg)
{
    vector<Point2f> object_2d_point;
        
    for (const auto &p2d : armor_identify_msg.armor_2d_points) {
        object_2d_point.emplace_back(Point2f(p2d.x, p2d.y));
    }
    Rect armorRect = boundingRect(object_2d_point);

    int min_x = armor_identify_msg.armor_2d_points[0].x;
    int min_y = armor_identify_msg.armor_2d_points[0].y;
    int max_x = armor_identify_msg.armor_2d_points[0].x;
    int max_y = armor_identify_msg.armor_2d_points[0].y;

    for (const auto& point : armor_identify_msg.armor_2d_points) {
        if (point.x < min_x) min_x = point.x;
        if (point.y < min_y) min_y = point.y;
        if (point.x > max_x) max_x = point.x;
        if (point.y > max_y) max_y = point.y;
    }
    
    identify_info_ptr->armor_width = max_x - min_x;
    identify_info_ptr->armor_height = max_y - min_y;

    identify_info_ptr->armor_center_x = armorRect.x + identify_info_ptr->armor_width * 0.5;
    identify_info_ptr->armor_center_y = armorRect.y + identify_info_ptr->armor_height * 0.5;
}