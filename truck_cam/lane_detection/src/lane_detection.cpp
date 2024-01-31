#include <include/lane_detection.hpp>

namespace Perception {

LaneDetector::LaneDetector()
        : Node("LaneDetector", rclcpp::NodeOptions()
                                .allow_undeclared_parameters(true)
                                .automatically_declare_parameters_from_overrides(true))
{
    /**************/
    /* ROS2 Topic */
    /**************/
    std::string SensorCamTopicName; // front cam
    int SensorCamQueueSize;
    
    
    
    /******************************/
    /* Ros Topic Subscribe Option */
    /******************************/
    this->get_parameter_or("",);

    /****************************/
    /* Ros Topic Publish Option */
    /****************************/


    /************************/
    /* Ros Topic Subscriber */
    /************************/


    /***********************/
    /* Ros Topic Publisher */
    /***********************/

    /***************/
    /* View Option */
    /***************/
    this->get_parameter_or("image_view/enable_opencv", viewImage_, true);
    this->get_parameter_or("image_view/wait_key_delay", waitKeyDelay_, 3);
    this->get_parameter_or("image_view/TEST", TEST, false);


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
    LoadParams();

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

    isNodeRunning_ = true;
    lanedetect_Thread = std::thread(&LaneDetector::lanedetectInThread, this);

}

LaneDetector::~LaneDetector(void)
{
    isNodeRunning_ = false;

    clear_release();
}

int LaneDetector::arrMaxIdx(int hist[], int start, int end, int Max) {
    int max_idx = -1;
    int max_val = 0;
    int min_pix = 30 * width_ / 1280;

    if (end > Max)
        end = Max;

    for (int i = start; i < end; i++) {
        if (max_val < hist[i]) {
        max_val = hist[i];
        max_idx = i;
        }
    }
    if ((max_idx == -1) || (hist[max_idx] < (size_t)min_pix)) {
    //    cout << "ERROR : hist range" << endl;
        return -1;
    }
    return max_idx;
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


Mat LaneDetector::polyfit(vector<int> x_val, vector<int> y_val) {
    Mat coef(3, 1, CV_32F);
    int i, j, k, n, N;
    N = (int)x_val.size();
    n = 2;
    double* x, * y;
    x = new double[N];
    y = new double[N];

    for (int q = 0; q < N; q++) {
        x[q] = (double)(x_val[q]);
        y[q] = (double)(y_val[q]);
    }
    
    double* X;
    X = new double[2 * n + 1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    
    for (i = 0; i < (2 * n + 1); i++)
    {
        X[i] = 0;
        for (j = 0; j < N; j++)
            X[i] = X[i] + pow(x[j], i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }
    
    double** B, * a;
    B = new double* [n + 1];
    
    for (int i = 0; i < (n + 1); i++)
        B[i] = new double[n + 2];
    
    a = new double[n + 1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    
    for (i = 0; i <= n; i++)
        for (j = 0; j <= n; j++)
            B[i][j] = X[i + j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    double* Y;
    Y = new double[n + 1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    for (i = 0; i < (n + 1); i++)
    {
        Y[i] = 0;
        for (j = 0; j < N; j++)
            Y[i] = Y[i] + pow(x[j], i) * y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
    for (i = 0; i <= n; i++)
        B[i][n + 1] = Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
    n = n + 1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations

    for (i = 0; i < n; i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k = i + 1; k < n; k++)
            if (B[i][i] < B[k][i])
                for (j = 0; j <= n; j++)
                {
                    double temp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = temp;
                }

    for (i = 0; i < (n - 1); i++)            //loop to perform the gauss elimination
        for (k = i + 1; k < n; k++)
        {
            double t = B[k][i] / B[i][i];
            for (j = 0; j <= n; j++)
                B[k][j] = B[k][j] - t * B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
        }
    for (i = n - 1; i >= 0; i--)                //back-substitution
    {                        //x is an array whose values correspond to the values of x,y,z..
        a[i] = B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
        for (j = 0; j < n; j++)
            if (j != i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                a[i] = a[i] - B[i][j] * a[j];
        a[i] = a[i] / B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
        coef.at<float>(i, 0) = (float)a[i];
    }

    delete[] x;
    delete[] y;
    delete[] X;
    delete[] Y;
    delete[] B;
    delete[] a;

    return coef; 

}


// C++ Numpy 있는지 확인 훨씬 빠름
// 해당 함수도 쪼개기
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

        for (size_t i = 0; i < maxIndices.size(); ++i) {
            if (maxIndices[i] == -1) {
                cluster_num = 2;
            }
            for (size_t j = i+1; j < maxIndices.size(); ++j) {
                if (maxIndices[i] != -1 && maxIndices[j] != -1) {
                    if (std::abs(maxIndices[i] - maxIndices[j]) <= 60) {
                        cluster_num = 2;
                        break
                    }
                }
            }

        }
        if (cluster_num == 2) {
            maxIndices = clusterHistogram(hist, cluster_num);
        }

        if (cluster_num == 3) {
            if (E_flag == true) {
                Llane_base = maxIndices[0];
                Rlane_base = maxIndices[1];
                Elane_base = maxIndices[2];
            }
            else if (E2_flag == true) {
                E2lane_base = maxIndices[0];
                Llane_base = maxIndices[1];
                Rlane_base = maxIndices[2];
            }
        }
        else if (cluster_num == 2) {
            if (E_flag == true) {
                Llane_base = -1;
                Rlane_base = maxIndices[0];
                Elane_bass = maxIndices[1];
            }
            else if (E2_flag == true) {
                E2lane_base = maxIndices[0];
                Llane_base = maxIndices[1];
                Rlane_base = -1;
            }
        }
    }

    int Llane_current = Llane_base;
    int Rlane_current = Rlane_base;
    int Elane_current = Elane_base;
    int E2lane_current = E2lane_base;

}

Mat LaneDetector::Image2BEV(Mat _frame, int _delay ,bool _view) {
    Mat test_trans, new_frame, gray_frame, binary_frame, overlap_frame, sliding_frame, resized_frame, warped_frame;
    static struct timeval startTime, endTime;
    static bool flag = false;
    static bool lc_flag = false;
    double diffTime = 0.0;


    // corners_, warpCorners_ 찾아야함
    Mat trans = getPerspectiveTransform(corners_, warpCorners_);

    if (!_frame.empty()) resize(_frame, new_frame, Size(width_, height_), 0, 0, cv::INTER_LINEAR)
    
    cuda::GpuMat gpu_frame, gpu_remap_frame, gpu_warped_frame, gpu_blur_frame, gpu_gray_frame, gpu_binary_frame, gpu_erode_frame;

    gpu_frame.upload(new_frame);
    // 깔삼한 용어 찾으면 col, row 수정
    cuda::remap(gpu_frame, gpu_remap_frame, gpu_, gpu_, INTER_LINEAR);
    gpu_remap_frame.download(new_frame);

    if (imageStatus_ && TEST) {
        cuda::warpPerspective(gpu_remap_frame, gpu_warped_frame, test_trans, Size(width_, height_));
    }

    else {
        cuda::warpPerspective(gpu_remap_frame, gpu_warped_frame, trans, Size(width_, height_));
    }
    gpu_warped_frame.download(warped_frame);

    static cv::Ptr<cv::cuda::Filter> filters;
    filters = cv::cuda::createGaussianFilter(gpu_warped_frame.type(), gpu_blur_frame.type(), cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);
    filters->apply(gpu_warped_frame, gpu_blur_frame);
    cuda::cvtColor(gpu_blur_frame, gpu_gray_frame, COLOR_BGR2GRAY);
    gpu_gray_frame.download(gray_frame);
    /* erode 연산 추가할거면 로직 수정 */
    for (int y = height_/2; y < gray_frame.rows; y++) {
        for (int x = 0; x < gray_frame.cols; x++) {
            if (gray_frame.at<uchar>(y, x) == 0) {
                gray_frame.at<uchar>(y, x) = 100;
            }
        }
    }

    /* Adpative Threshold -> 해당 로직도 8bit -> 1bit로 변환 가능 */
    adaptiveThreshold(gray_frame, binary_frame, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, Threshold_box_size_, -(Threshold_box_offset_));

    return binary_frame;
}


// adaptive tau 고려해봐도?
float LaneDetector::lowPassFilter(double sampling_time, float est_value, float prev_res) {
    float res = 0;
    float tau = 0.1f;
    double st = 0.0;

    if (sampling_time > 1.0) st = 1.0;
    else st = sampling_time;
    res = ((tau * prev_res) + (st * est_value)) / (tau + st);

    return res
}

Point LaneDetector::warpPoint() {

}

Mat LaneDetector::draw_lane() {

}

void LaneDetector::clear_release() {

}


}