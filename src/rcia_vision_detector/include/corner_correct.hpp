#ifndef CORNER_CORRECT_HPP_
#define CORNER_CORRECT_HPP_

#include <opencv2/opencv.hpp>
#include <utility>

//main输入(灰度  左右灯条宽度 最小正矩形四个顶点) 
// (*最小正矩形四个顶点+灰度)->(对称轴+平均亮度)->角点坐标

// 定义对称轴结构体，用于存储灯光的质心、对称轴方向和平均亮度
struct SymmetryAxis {
    cv::Point2f centroid;  // 质心坐标
    cv::Point2f direction; // 对称轴方向
    float mean_val;        // 灯光区域的平均亮度
  };
  
  // LightCornerCorrector 类用于提高灯光角点的精度
  // 主要通过 PCA 算法计算灯光的对称轴，并基于亮度梯度搜索角点
  class LightCornerCorrector {
  public:
    // 构造函数，无参数
    explicit LightCornerCorrector() noexcept {}
  
    // 修正装甲灯光的角点

    //输入左右灯条宽度和灰度图像

    // 输出：修正后的装甲灯光角点
    std::pair<std::vector<cv::Point>, std::string> correctCorners(std::vector<cv::Point> first_contour,std::vector<cv::Point> second_contour, const cv::Mat &gray_img);
  
  private:
    // 计算灯光的对称轴
 
    //输入 灰度图 旋转矩形的最小正矩形四个顶点 
  
    // 输出：包含质心、方向和平均亮度的对称轴信息
    SymmetryAxis findSymmetryAxis(const cv::Mat &gray_img,cv::Rect light_box);
  
    // 搜索结果灯光的角点

    //输入 灰度图 灯条长度 对称轴 平均亮度 角点方向（top 或 bottom）
  
    // 输出：找到的角点坐标，若未找到则返回 (-1, -1)
    cv::Point2f findCorner(const cv::Mat &gray_img,
                           const cv::RotatedRect minRect,
                           const SymmetryAxis &axis,
                           std::string order);
  };

#endif
