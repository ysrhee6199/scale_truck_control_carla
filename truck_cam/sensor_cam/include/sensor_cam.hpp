#pragma once

// C++
#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <time.h>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <algorithm>
#include <limits>
#include <random>
#include <condition_variable>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudafilters.hpp>

//ROS2

#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/xav2lane.hpp"
#include "ros2_msg/msg/lane2xav.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono_literals;


namespace Sensor {
class SensorCam : public rclcpp::Node // ROS2?
{
public:
    SensorCam();
    ~SensorCam();

    //Timer
    struct timeval start_, end_;
    float display_img(Mat _frame, int _delay, bool _veiw);
    Mat frame_;

// private로 선언해도 되나?
private:
    void LoadParmas(void);

    //Subscriber
    rclcpp::Subscrioption<sensor_msgs::msg::Image>::SharedPtr ImageSubscriber_;
    rclcpp::Subscrioption<sensor_msgs::msg::Image>::SharedPtr rearImageSubscriber_;
    
    //Callback FUnc
    void ImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void rearImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    bool viewImage_;
    int waitkeyDelay_;
    bool isNodeRunning_ = true;


    //image
    bool imageStatus_ = false;
    std_msgs::msg::Header imageHeader_;
    cv::Mat camImageCopy_;
    float AngleDegree_;
    cv::Mat prev_frame, prev2_frame;
    cv::Mat cluster_frame;

    //rear
    bool rear_view_ = false;
    bool rearImageSTatus_ = false;
    std_msgs::msg::Header rearImageHeader_;
    cv::Mat rearCamImageCopy_;
    

    /* Callback SYnchronisation */
    mutex cam_mutex;
    bool cam_new_frame_arrived;
    condition_variable cam_condition_variable;
  
    /*  Callback Synchronisation  (Rear)  */
    mutex rear_cam_mutex;
    bool rear_cam_new_frame_arrived;
    condition_variable rear_cam_condition_variable;
    /********** Camera calibration **********/
    Mat f_camera_matrix, f_dist_coeffs;
    Mat r_camera_matrix, r_dist_coeffs;
    Mat map1_, map2_, f_map1_, f_map2_, r_map1_, r_map2_;
    
}
}