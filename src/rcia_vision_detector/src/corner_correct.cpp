#include "corner_correct.hpp"

#include <numeric>
#include <utility>
#include <algorithm> // 包含 swap 函数的头文件

// 修正灯光角点的主函数
//输入左右灯条宽度
std::pair<std::vector<cv::Point>, std::string> LightCornerCorrector::correctCorners(std::vector<cv::Point> first_contour, std::vector<cv::Point> second_contour, const cv::Mat &gray_img) {
  constexpr int PASS_OPTIMIZE_WIDTH = 3;
  std::vector<cv::Point> corners;
  std::string status = "False";

  // 处理左侧灯光
  cv::Rect left_light_box = cv::boundingRect(first_contour);
  cv::RotatedRect left_minRect = cv::minAreaRect(first_contour);

  if (left_minRect.size.width > left_minRect.size.height) {
      std::swap(left_minRect.size.width, left_minRect.size.height);
  }

  bool leftTopFound = false, leftBottomFound = false;
  cv::Point2f LT, LB;

  if (left_minRect.size.width > PASS_OPTIMIZE_WIDTH) {
      // 计算左侧灯光的对称轴
      SymmetryAxis left_axis = findSymmetryAxis(gray_img, left_light_box);

      // 查找左侧灯光的顶部角点
      if (cv::Point2f t = findCorner(gray_img, left_minRect, left_axis, "top"); t.x > 0) {
          LT = t;
          leftTopFound = true;
      }
      // 查找左侧灯光的底部角点
      if (cv::Point2f b = findCorner(gray_img, left_minRect, left_axis, "bottom"); b.x > 0) {
          LB = b;
          leftBottomFound = true;
      }
  }

  // 处理右侧灯光
  cv::Rect right_light_box = cv::boundingRect(second_contour);
  cv::RotatedRect right_minRect = cv::minAreaRect(second_contour);

  if (right_minRect.size.width > right_minRect.size.height) {
      std::swap(right_minRect.size.width, right_minRect.size.height);
  }

  bool rightTopFound = false, rightBottomFound = false;
  cv::Point2f RT, RB;

  if (right_minRect.size.width > PASS_OPTIMIZE_WIDTH) {
      // 计算右侧灯光的对称轴
      SymmetryAxis right_axis = findSymmetryAxis(gray_img, right_light_box);

      // 查找右侧灯光的顶部角点
      if (cv::Point2f t = findCorner(gray_img, right_minRect, right_axis, "top"); t.x > 0) {
          RT = t;
          rightTopFound = true;
      }
      // 查找右侧灯光的底部角点
      if (cv::Point2f b = findCorner(gray_img, right_minRect, right_axis, "bottom"); b.x > 0) {
          RB = b;
          rightBottomFound = true;
      }
  }

  // 检查四个角点是否都找到
  if (leftTopFound && rightTopFound && rightBottomFound && leftBottomFound) {
      corners.emplace_back(LT);
      corners.emplace_back(RT);
      corners.emplace_back(RB);
      corners.emplace_back(LB);
      status = "True";
  } else {
      // 如果有任何一个点未找到，返回四个零点
      corners = {cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0)};
  }

  return {corners, status};
}


// 计算灯光的对称轴
//输入 灰度图 旋转矩形的最小正矩形 
SymmetryAxis LightCornerCorrector::findSymmetryAxis(const cv::Mat &gray_img, cv::Rect light_box) {
  // 定义最大亮度和扩展比例
  constexpr float MAX_BRIGHTNESS = 25;
  constexpr float SCALE = 0.07;

  // 获取灯光区域的边界框并扩展
  // 获取包含旋转矩形的最小正矩形
  // cv::Rect light_box = boundingRect(light_contour);
  light_box.x -= light_box.width * SCALE;
  light_box.y -= light_box.height * SCALE;
  light_box.width += light_box.width * SCALE * 2;
  light_box.height += light_box.height * SCALE * 2;

  // 确保扩展后的边界框在图像范围内
  light_box.x = std::max(light_box.x, 0);
  light_box.x = std::min(light_box.x, gray_img.cols - 1);
  light_box.y = std::max(light_box.y, 0);
  light_box.y = std::min(light_box.y, gray_img.rows - 1);
  light_box.width = std::min(light_box.width, gray_img.cols - light_box.x);
  light_box.height = std::min(light_box.height, gray_img.rows - light_box.y);

  // 提取边界框内的图像区域并归一化
  cv::Mat roi = gray_img(light_box);
  float mean_val = cv::mean(roi)[0];
  roi.convertTo(roi, CV_32F);
  cv::normalize(roi, roi, 0, MAX_BRIGHTNESS, cv::NORM_MINMAX);

  // 计算质心
  cv::Moments moments = cv::moments(roi, false);
  cv::Point2f centroid = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00) +
                         cv::Point2f(light_box.x, light_box.y);

  // 构建点云数据
  std::vector<cv::Point2f> points;
  for (int i = 0; i < roi.rows; i++) {
    for (int j = 0; j < roi.cols; j++) {
      for (int k = 0; k < std::round(roi.at<float>(i, j)); k++) {
        points.emplace_back(cv::Point2f(j, i));
      }
    }
  }
  cv::Mat points_mat = cv::Mat(points).reshape(1);

  // 使用 PCA 提取主方向
  auto pca = cv::PCA(points_mat, cv::Mat(), cv::PCA::DATA_AS_ROW);

  // 提取对称轴方向并归一化
  cv::Point2f axis =
    cv::Point2f(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
  axis = axis / cv::norm(axis);

  // 确保对称轴方向朝下
  if (axis.y > 0) {
    axis = -axis;
  }

  // 返回对称轴信息
  return SymmetryAxis{.centroid = centroid, .direction = axis, .mean_val = mean_val};
}

// 搜索框灯光的角点
//输入 灰度图 灯条长度宽度 对称轴 平均亮度
cv::Point2f LightCornerCorrector::findCorner(const cv::Mat &gray_img,
                                             const cv::RotatedRect minRect,
                                             const SymmetryAxis &axis,
                                             std::string order) {
  // 定义搜索的起始和结束比例
  constexpr float START = 0.8 / 2;
  constexpr float END = 1.2 / 2;

  // 检查点是否在图像范围内
  auto inImage = [&gray_img](const cv::Point &point) -> bool {
    return point.x >= 0 && point.x < gray_img.cols && point.y >= 0 && point.y < gray_img.rows;
  };

  // 计算两点之间的欧几里得距离
  auto distance = [](float x0, float y0, float x1, float y1) -> float {
    return std::sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
  };

  // 根据角点方向确定搜索方向
  int oper = order == "top" ? 1 : -1;
  float L = minRect.size.height;
  //对称轴方向
  float dx = axis.direction.x * oper;
  float dy = axis.direction.y * oper;

  // 存储候选角点
  std::vector<cv::Point2f> candidates;

  // 搜索框多个候选角点
  int n = minRect.size.width - 2;
  int half_n = std::round(n / 2);
  for (int i = -half_n; i <= half_n; i++) {
    // 计算搜索的起始点
    float x0 = axis.centroid.x + L * START * dx + i;
    float y0 = axis.centroid.y + L * START * dy;

    // 初始化变量
    cv::Point2f prev = cv::Point2f(x0, y0);
    cv::Point2f corner = cv::Point2f(x0, y0);
    float max_brightness_diff = 0;
    bool has_corner = false;

    // 沿对称轴方向搜索角点
    for (float x = x0 + dx, y = y0 + dy; distance(x, y, x0, y0) < L * (END - START);
         x += dx, y += dy) {
      cv::Point2f cur = cv::Point2f(x, y);
      if (!inImage(cv::Point(cur))) {
        break;
      }

      // 计算亮度差
      float brightness_diff = gray_img.at<uchar>(prev) - gray_img.at<uchar>(cur);
      if (brightness_diff > max_brightness_diff && gray_img.at<uchar>(prev) > axis.mean_val) {
        max_brightness_diff = brightness_diff;
        corner = prev;
        has_corner = true;
      }

      prev = cur;
    }

    // 如果找到角点，添加到候选列表
    if (has_corner) {
      candidates.emplace_back(corner);
    }
  }

  // 如果有候选角点，返回平均值作为最终角点
  if (!candidates.empty()) {
    cv::Point2f result = std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0));
    return result / static_cast<float>(candidates.size());
  }

  // 如果没有找到角点，返回无效坐标
  return cv::Point2f(-1, -1);
}