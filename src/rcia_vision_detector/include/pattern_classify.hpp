#include "vision_initialize.hpp"

#include <opencv2/dnn.hpp>

#include "Types.hpp"


class PatternClassifier {
public:

    PatternClassifier();

    void getSimpleRoiPattern(const vector<Point> &Draw_Box, const Mat &img, string &armor_type, int &find_count, int& armorPatternIdx, double& armorPatternAcc);

    Mat getPatternRoi(const Mat &Simple_Roi_Pattern);

    void getPatternClassify(const Mat &PatternRoi, int &PatternIdx, double &PatternAcc, int &find_count);

    
private:

    cv::dnn::Net net;

    bool CheckFallbackCondition(int find_count);
    void UpdateHistory(int idx, double acc);

    int lastPatternIdx;
    double lastPatternAcc;

};
