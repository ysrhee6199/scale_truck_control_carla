#include <include/lane_detection.hpp>

namespace LaneDetector {

LaneDetector::LaneDetector()
{
    /**************/
    /* ROS2 Topic */
    /**************/
    
    
    
    
    /******************************/
    /* Ros Topic Subscribe Option */
    /******************************/


    /****************************/
    /* Ros Topic Publish Option */
    /****************************/


    /************************/
    /* Ros Topic Subscriber */
    /************************/


    /***********************/
    /* Ros Topic Publisher */
    /***********************/


    /* Image2BEV parameter */
    prev_err_ = 0;

    last_Llane_base_ = 0;
    last_Rlane_base_ = 0;
    left_coef_ = Mat::zeros(3, 1, CV_32F);
    right_coef_ = Mat::zeros(3, 1, CV_32F);
    center_coef_ = Mat::zeros(3, 1, CV_32F);
    center2_coef_ = Mat::zeros(3, 1, CV_32F);
    center3_coef_ = Mat::zeros(3, 1, CV_32F);
    extra_coef_ = Mat::zeros(3, 1, CV_32F);
    extra2_coef_ = Mat::zeros(3, 1, CV_32F);

    this->get_parameter_or("ROI/width", width_, 640);
    this->get_parameter_or("ROI/height", height_, 480);
    center_position_ = width_/2;

    float t_gap[5], b_gap[5], t_height[5], b_height[5], f_extra[5], b_extra[5];
    int top_gap[5], bot_gap[5], top_height[5], bot_height[5], extra_up[5], extra_down[5];
    float wide_extra_upside_[5], wide_extra_downside_[5];

    this->get_parameter_or("ROI/dynamic_roi",option_, true);
    this->get_parameter_or("ROI/threshold",threshold_, 128);
    this->get_parameter_or("ROI/frontRoi_ratio",frontRoi_ratio, 204.0f);
    this->get_parameter_or("ROI/rearRoi_ratio",rearRoi_ratio, 210.0f);

    this->get_parameter_or("ROI/front_cam/top_gap",t_gap[0], 0.336f);
    this->get_parameter_or("ROI/front_cam/bot_gap",b_gap[0], 0.078f);
    this->get_parameter_or("ROI/front_cam/top_height",t_height[0], 0.903f);
    this->get_parameter_or("ROI/front_cam/bot_height",b_height[0], 0.528f);
    this->get_parameter_or("ROI/front_cam/extra_f",f_extra[0], 0.0f);
    this->get_parameter_or("ROI/front_cam/extra_b",b_extra[0], 0.0f);
    this->get_parameter_or("ROI/front_cam/extra_up",extra_up[0], 0);
    this->get_parameter_or("ROI/front_cam/extra_down",extra_down[0], 0);

    this->get_parameter_or("ROI/wide_right/top_gap",t_gap[1], 0.886f);
    this->get_parameter_or("ROI/wide_right/bot_gap",b_gap[1], 0.078f);
    this->get_parameter_or("ROI/wide_right/top_height",t_height[1], 0.903f);
    this->get_parameter_or("ROI/wide_right/bot_height",b_height[1], 0.528f);
    this->get_parameter_or("ROI/wide_right/extra_f",f_extra[1], 0.0f);
    this->get_parameter_or("ROI/wide_right/extra_b",b_extra[1], 0.0f);
    this->get_parameter_or("ROI/wide_right/extra_up",extra_up[1], 0);
    this->get_parameter_or("ROI/wide_right/extra_down",extra_down[1], 0);

    this->get_parameter_or("ROI/wide_left/top_gap",t_gap[2], 0.886f);
    this->get_parameter_or("ROI/wide_left/bot_gap",b_gap[2], 0.078f);
    this->get_parameter_or("ROI/wide_left/top_height",t_height[2], 0.903f);
    this->get_parameter_or("ROI/wide_left/bot_height",b_height[2], 0.528f);
    this->get_parameter_or("ROI/wide_left/extra_f",f_extra[2], 0.0f);
    this->get_parameter_or("ROI/wide_left/extra_b",b_extra[2], 0.0f);
    this->get_parameter_or("ROI/wide_left/extra_up",extra_up[2], 0);
    this->get_parameter_or("ROI/wide_left/extra_down",extra_down[2], 0);

    this->get_parameter_or("ROI/rear_cam/top_gap",t_gap[3], 0.405f);
    this->get_parameter_or("ROI/rear_cam/bot_gap",b_gap[3], 0.17f);
    this->get_parameter_or("ROI/rear_cam/top_height",t_height[3], 0.99f);
    this->get_parameter_or("ROI/rear_cam/bot_height",b_height[3], 0.47f);
    this->get_parameter_or("ROI/rear_cam/extra_f",f_extra[3], 1.0f);
    this->get_parameter_or("ROI/rear_cam/extra_b",b_extra[3], 10.0f);
    this->get_parameter_or("ROI/rear_cam/extra_up",extra_up[3], 140);
    this->get_parameter_or("ROI/rear_cam/extra_down",extra_down[3], 180);

    this->get_parameter_or("ROI/test/top_gap",t_gap[4], 0.405f);
    this->get_parameter_or("ROI/test/bot_gap",b_gap[4], 0.17f);
    this->get_parameter_or("ROI/test/top_height",t_height[4], 0.99f);
    this->get_parameter_or("ROI/test/bot_height",b_height[4], 0.47f);
    this->get_parameter_or("ROI/test/extra_f",f_extra[4], 1.0f);
    this->get_parameter_or("ROI/test/extra_b",b_extra[4], 10.0f);
    this->get_parameter_or("ROI/test/extra_up",extra_up[4], 140);
    this->get_parameter_or("ROI/test/extra_down",extra_down[4], 180);

    this->get_parameter_or("threshold/box_size", Threshold_box_size_, 51);
    this->get_parameter_or("threshold/box_offset", Threshold_box_offset_, 50);

    distance_ = 0;

    corners_.resize(4);
    warpCorners_.resize(4);

    test_corners_.resize(4);
    test_warpCorners_.resize(4);

    e_values_.resize(3);

    /*** front cam ROI setting ***/
    fROIcorners_.resize(4);
    fROIwarpCorners_.resize(4);

    top_gap[0] = width_ * t_gap[0]; 
    bot_gap[0] = width_ * b_gap[0];
    top_height[0] = height_ * t_height[0];
    bot_height[0] = height_ * b_height[0];

    fROIcorners_[0] = Point2f(top_gap[0]+f_extra[0], bot_height[0]);
    fROIcorners_[1] = Point2f((width_ - top_gap[0])+f_extra[0], bot_height[0]);
    fROIcorners_[2] = Point2f(bot_gap[0]+b_extra[0], top_height[0]);
    fROIcorners_[3] = Point2f((width_ - bot_gap[0])+b_extra[0], top_height[0]);
    
    wide_extra_upside_[0] = extra_up[0];
    wide_extra_downside_[0] = extra_down[0];
    
    fROIwarpCorners_[0] = Point2f(wide_extra_upside_[0], 0.0);
    fROIwarpCorners_[1] = Point2f(width_ - wide_extra_upside_[0], 0.0);
    fROIwarpCorners_[2] = Point2f(wide_extra_downside_[0], height_);
    fROIwarpCorners_[3] = Point2f(width_ - wide_extra_downside_[0], height_);
    /*** front cam ROI setting ***/

    /*** Wide Right ROI setting ***/
    rROIcorners_.resize(4);
    rROIwarpCorners_.resize(4);

    top_gap[1] = width_ * t_gap[1];
    bot_gap[1] = width_ * b_gap[1];
    top_height[1] = height_ * t_height[1];
    bot_height[1] = height_ * b_height[1];

    rROIcorners_[0] = Point2f(top_gap[1]+f_extra[1], bot_height[1]);
    rROIcorners_[1] = Point2f((width_ - top_gap[1])+f_extra[1], bot_height[1]);
    rROIcorners_[2] = Point2f(bot_gap[1]+b_extra[1], top_height[1]);
    rROIcorners_[3] = Point2f((width_ - bot_gap[1])+b_extra[1], top_height[1]);

    wide_extra_upside_[1] = extra_up[1];
    wide_extra_downside_[1] = extra_down[1];

    rROIwarpCorners_[0] = Point2f(wide_extra_upside_[1], 0.0);
    rROIwarpCorners_[1] = Point2f(width_ - wide_extra_upside_[1], 0.0);
    rROIwarpCorners_[2] = Point2f(wide_extra_downside_[1], height_);
    rROIwarpCorners_[3] = Point2f(width_ - wide_extra_downside_[1], height_);
    /*** Wide Right ROI setting ***/

    /*** Wide left ROI setting ***/
    lROIcorners_.resize(4);
    lROIwarpCorners_.resize(4);

    top_gap[2] = width_ * t_gap[2];
    bot_gap[2] = width_ * b_gap[2];
    top_height[2] = height_ * t_height[2];
    bot_height[2] = height_ * b_height[2];

    lROIcorners_[0] = Point2f(top_gap[2]+f_extra[2], bot_height[2]);
    lROIcorners_[1] = Point2f((width_ - top_gap[2])+f_extra[2], bot_height[2]);
    lROIcorners_[2] = Point2f(bot_gap[2]+b_extra[2], top_height[2]);
    lROIcorners_[3] = Point2f((width_ - bot_gap[2])+b_extra[2], top_height[2]);

    wide_extra_upside_[2] = extra_up[2];
    wide_extra_downside_[2] = extra_down[2];

    lROIwarpCorners_[0] = Point2f(wide_extra_upside_[2], 0.0);
    lROIwarpCorners_[1] = Point2f(width_ - wide_extra_upside_[2], 0.0);
    lROIwarpCorners_[2] = Point2f(wide_extra_downside_[2], height_);
    lROIwarpCorners_[3] = Point2f(width_ - wide_extra_downside_[2], height_);
    /*** Wide left ROI setting ***/

    /*** rear cam ROI setting ***/
    rearROIcorners_.resize(4);
    rearROIwarpCorners_.resize(4);

    top_gap[3] = width_ * t_gap[3]; 
    bot_gap[3] = width_ * b_gap[3];
    top_height[3] = height_ * t_height[3];
    bot_height[3] = height_ * b_height[3];

    rearROIcorners_[0] = Point2f(top_gap[3]+f_extra[3], bot_height[3]);
    rearROIcorners_[1] = Point2f((width_ - top_gap[3])+f_extra[3], bot_height[3]);
    rearROIcorners_[2] = Point2f(bot_gap[3]+b_extra[3], top_height[3]);
    rearROIcorners_[3] = Point2f((width_ - bot_gap[3])+b_extra[3], top_height[3]);
    
    wide_extra_upside_[3] = extra_up[3];
    wide_extra_downside_[3] = extra_down[3];
    
    rearROIwarpCorners_[0] = Point2f(wide_extra_upside_[3], 0.0);
    rearROIwarpCorners_[1] = Point2f(width_ - wide_extra_upside_[3], 0.0);
    rearROIwarpCorners_[2] = Point2f(wide_extra_downside_[3], height_);
    rearROIwarpCorners_[3] = Point2f(width_ - wide_extra_downside_[3], height_);
    /*** rear cam ROI setting ***/

    /*** front cam ROI 2.5m setting ***/
    testROIcorners_.resize(4);
    testROIwarpCorners_.resize(4);

    top_gap[4] = width_ * t_gap[4]; 
    bot_gap[4] = width_ * b_gap[4];
    top_height[4] = height_ * t_height[4];
    bot_height[4] = height_ * b_height[4];

    testROIcorners_[0] = Point2f(top_gap[4]+f_extra[4], bot_height[4]);
    testROIcorners_[1] = Point2f((width_ - top_gap[4])+f_extra[4], bot_height[4]);
    testROIcorners_[2] = Point2f(bot_gap[4]+b_extra[4], top_height[4]);
    testROIcorners_[3] = Point2f((width_ - bot_gap[4])+b_extra[4], top_height[4]);
    
    wide_extra_upside_[4] = extra_up[4];
    wide_extra_downside_[4] = extra_down[4];
    
    testROIwarpCorners_[0] = Point2f(wide_extra_upside_[4], 0.0);
    testROIwarpCorners_[1] = Point2f(width_ - wide_extra_upside_[4], 0.0);
    testROIwarpCorners_[2] = Point2f(wide_extra_downside_[4], height_);
    testROIwarpCorners_[3] = Point2f(width_ - wide_extra_downside_[4], height_);
    /*** front cam ROI 2.5m setting ***/   

}

LaneDetector::~LaneDetector(void)
{

}

int LaneDetector::arrMaxIdx(int hist[], int start, int end, int Max) {
    int max_index = -1;
    int max_val = 0;
    int min_pix = 30 * width_ / 1280;

    if (end > Max)
        end = Max;

    for (int i = start; i < end; i++) {
        if (max_val < hist[i]) {
        max_val = hist[i];
        max_index = i;
        }
    }
    if ((max_index == -1) || (hist[max_index] < (size_t)min_pix)) {
    //    cout << "ERROR : hist range" << endl;
        return -1;
    }
    return max_index;
}



// 해당 로직이 왜 들어갔지? 일단 여기도 최적화 가능함
std::vector<int> LaneDetector::ClusterHistogram(int* hist, int cluster_num) {
    struct Cluster {
        int centerIndex;
        int maxValue;
        int maxValueIndex;
    };

    std::vector<cv::Point2f> points;
    for (int i = 0; i < width_; ++i) {
        for (int j = 0; j < hist[i]; ++j) {
            points.push_back(cv::Point2f(i, j));
        }
    }

    // K-means cluster
    cv::Mat labels, centers;
    try
    {
        cv::kmeans(points, cluster_num, labels, 
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
                    3, cv::KMEANS_PP_CENTERS, centers);
    } catch (const cv::Exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        std::cerr << "The error occured in cv::kmeans()" << std::endl;
        std::vector<int> result;
        for (int i = 0; i < cluster_num; i++) {
            result.push_back(-1);
        }
        return result;
    }

}


Mat LaneDetector::polyfit() {

}


// C++ Numpy 있는지 확인 훨씬 빠름
Mat LaneDetector::detect_lines_sliding_window(Mat _frame, bool _view) {
    Mat frame, result;
    int width = frame.cols
    int height = frame.rows

    _frame.copyTo(frame);
    Mat nonZero;
    findNonZero(frame, nonZero);

    std::vector<int> good_left_idxs;
    std::vector<int> good_right_idxs;
    std::vector<int> good_extra_idxs;
    std::vector<int> good_extra2_idxs;

    int* hist = new int[width];


    // initialization hist
    for (int i = 0; i < width; i++) {
        hist[i] += 1;
    }
    // 0~255 말고 0~1로 변경하면 8bit -> 1bit로 줄일 수 있을듯?
    for (int j = height/2; j < height; j++) {
        for (int i = 0; i < width; i++) {
            if (frame.at <uchar>(j,i) == 255) {
                hist[i] += 1;
            }
        }
    }

    cvtColor(frame, result, COLOR_GRAY2BGR);

    // 곡률이 엄청 큰경우 해당 로직은 고장남, 추후에 고민 할 사람 있으면 고치세요.
    int mid_point = width / 2; // 320
    int n_windows = 9;
    int margin = 120 * width / 1280;

    int min_pix = 100 * width / 1280;

    int window_width = margin * 2; // 120
    int window_height;
    int distance;
    L_flag = true;
    R_flag = true;
    E_flag = true;
    E2_flag = true;
    if(!lc_right_flag_) E_flag = false;
    if(!lc_left_flag_) E2_flag = false;

    if (option_) {
        window_height = (height >= distance_) ? ((height-distance_) / n_windows) : (height / n_windows);
        distance = distance_; // sensor depth estimation
    } else {
        distance = 0;
        window_height = height / n_windows;
    }

    int Llane_base = arrMaxIdx(hist, 100, mid_point, width); // start ~ mid
    int Rlane_base = arrMaxIdx(hist, mid_point, width-100, width); // mid ~ w-100

    int E2lane_base = 0, Elane_base = 0;
    int cluster_num = 2;
    std::vector<int> maxIndices;

    if (E_flag == true || E2_flag == true) {
        cluster_num 3;
        maxIndices = clusterHistogram(hist, cluster_num);
    }



}

float LaneDetector::lowPassFilter() {

}

Point LaneDetector::warpPoint() {

}

Mat LaneDetector::draw_lane() {

}

void LaneDetector::clear_release() {

}


}