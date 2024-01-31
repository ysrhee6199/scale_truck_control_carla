#include <include/sensor_cam.hpp>


/* Camera Calibration */

namespace Sensor {

SensorCam::SensorCam()
        :Node("SensorCam", rclcpp::NodeOptions()
                          .allow_undeclared_parameters(true)
                          .automatically_declare_parameters_from_overrides(true))

{
 /**************/
  /* ROS2 Topic */
  /**************/
  std::string XavSubTopicName;
  int XavSubQueueSize;
  std::string ImageSubTopicName;
  int ImageSubQueueSize;
  std::string rearImageSubTopicName;
  int rearImageSubQueueSize;

  std::string XavPubTopicName;
  int XavPubQueueSize;

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  this->get_parameter_or("subscribers/xavier_to_lane/topic", XavSubTopicName, std::string("xav2lane_msg"));
  this->get_parameter_or("subscribers/xavier_to_lane/queue_size", XavSubQueueSize, 1);
  this->get_parameter_or("subscribers/image_to_lane/topic", ImageSubTopicName, std::string("usb_cam/image_raw"));
  this->get_parameter_or("subscribers/image_to_lane/queue_size", ImageSubQueueSize, 1);
  this->get_parameter_or("subscribers/rearImage_to_lane/topic", rearImageSubTopicName, std::string("rear_cam/image_raw"));
  this->get_parameter_or("subscribers/rearImage_to_lane/queue_size", rearImageSubQueueSize, 1);

  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  this->get_parameter_or("publishers/lane_to_xavier/topic", XavPubTopicName, std::string("lane2xav_msg"));
  this->get_parameter_or("publishers/lane_to_xavier/queue_size", XavPubQueueSize, 1);
  
  /************************/
  /* Ros Topic Subscriber */
  /************************/
  XavSubscriber_ = this->create_subscription<ros2_msg::msg::Xav2lane>(XavSubTopicName, XavSubQueueSize, std::bind(&LaneDetector::XavSubCallback, this, std::placeholders::_1));

  ImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(ImageSubTopicName, ImageSubQueueSize, std::bind(&LaneDetector::ImageSubCallback, this, std::placeholders::_1));

  rearImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(rearImageSubTopicName, rearImageSubQueueSize, std::bind(&LaneDetector::rearImageSubCallback, this, std::placeholders::_1));

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  XavPublisher_ = this->create_publisher<ros2_msg::msg::Lane2xav>(XavPubTopicName, XavPubQueueSize);

  /***************/
  /* View Option */
  /***************/

  /******* recording log *******/    
  gettimeofday(&start_, NULL);

      /******* Camera  calibration *******/
  double f_matrix[9], f_dist_coef[5], r_matrix[9], r_dist_coef[5];
  this->get_parameter_or("Calibration/f_matrix/a",f_matrix[0], 3.2918100682757097e+02);
  this->get_parameter_or("Calibration/f_matrix/b",f_matrix[1], 0.);
  this->get_parameter_or("Calibration/f_matrix/c",f_matrix[2], 320.);
  this->get_parameter_or("Calibration/f_matrix/d",f_matrix[3], 0.);
  this->get_parameter_or("Calibration/f_matrix/e",f_matrix[4], 3.2918100682757097e+02);
  this->get_parameter_or("Calibration/f_matrix/f",f_matrix[5], 240.);
  this->get_parameter_or("Calibration/f_matrix/g",f_matrix[6], 0.);
  this->get_parameter_or("Calibration/f_matrix/h",f_matrix[7], 0.);
  this->get_parameter_or("Calibration/f_matrix/i",f_matrix[8], 1.);

  this->get_parameter_or("Calibration/f_dist_coef/a",f_dist_coef[0], -3.2566540239089398e-01);
  this->get_parameter_or("Calibration/f_dist_coef/b",f_dist_coef[1], 1.1504807178349362e-01);
  this->get_parameter_or("Calibration/f_dist_coef/c",f_dist_coef[2], 0.);
  this->get_parameter_or("Calibration/f_dist_coef/d",f_dist_coef[3], 0.);
  this->get_parameter_or("Calibration/f_dist_coef/e",f_dist_coef[4], -2.1908791800876997e-02);

  this->get_parameter_or("Calibration/r_matrix/a",r_matrix[0], 326.31389227574556);
  this->get_parameter_or("Calibration/r_matrix/b",r_matrix[1], 0.);
  this->get_parameter_or("Calibration/r_matrix/c",r_matrix[2], 320.);
  this->get_parameter_or("Calibration/r_matrix/d",r_matrix[3], 0.);
  this->get_parameter_or("Calibration/r_matrix/e",r_matrix[4], 326.31389227574556);
  this->get_parameter_or("Calibration/r_matrix/f",r_matrix[5], 240.);
  this->get_parameter_or("Calibration/r_matrix/g",r_matrix[6], 0.);
  this->get_parameter_or("Calibration/r_matrix/h",r_matrix[7], 0.);
  this->get_parameter_or("Calibration/r_matrix/i",r_matrix[8], 1.);

  this->get_parameter_or("Calibration/r_dist_coef/a",r_dist_coef[0], -0.33295846454356126);
  this->get_parameter_or("Calibration/r_dist_coef/b",r_dist_coef[1], 0.12386827336557986);
  this->get_parameter_or("Calibration/r_dist_coef/c",r_dist_coef[2], 0.);
  this->get_parameter_or("Calibration/r_dist_coef/d",r_dist_coef[3], 0.);
  this->get_parameter_or("Calibration/r_dist_coef/e",r_dist_coef[4], -0.022565312043601477);
  LoadParmas();

  /*** front cam calibration  ***/
  f_camera_matrix = Mat::eye(3, 3, CV_64FC1);
  f_dist_coeffs = Mat::zeros(1, 5, CV_64FC1);
  f_camera_matrix = (Mat1d(3, 3) << f_matrix[0], f_matrix[1], f_matrix[2], f_matrix[3], f_matrix[4], f_matrix[5], f_matrix[6], f_matrix[7], f_matrix[8]);
  f_dist_coeffs = (Mat1d(1, 5) << f_dist_coef[0], f_dist_coef[1], f_dist_coef[2], f_dist_coef[3], f_dist_coef[4]);
  initUndistortRectifyMap(f_camera_matrix, f_dist_coeffs, Mat(), f_camera_matrix, Size(640, 480), CV_32FC1, f_map1_, f_map2_);

  /*** rear cam calibration  ***/
  r_camera_matrix = Mat::eye(3, 3, CV_64FC1);
  r_dist_coeffs = Mat::zeros(1, 5, CV_64FC1);
  r_camera_matrix = (Mat1d(3, 3) << r_matrix[0], r_matrix[1], r_matrix[2], r_matrix[3], r_matrix[4], r_matrix[5], r_matrix[6], r_matrix[7], r_matrix[8]);
  r_dist_coeffs = (Mat1d(1, 5) << r_dist_coef[0], r_dist_coef[1], r_dist_coef[2], r_dist_coef[3], r_dist_coef[4]);
  initUndistortRectifyMap(r_camera_matrix, r_dist_coeffs, Mat(), r_camera_matrix, Size(640, 480), CV_32FC1, r_map1_, r_map2_);

  map1_ = f_map1_.clone();
  map2_ = f_map2_.clone();

  isNodeRunning_ = true;
  // lanedetect_Thread = std::thread(&LaneDetector::lanedetectInThread, this);
  
}

SensorCam::~SensorCam(void)
{
    isNodeRunning_ = false;

    /* Unblock the other thread to shutdown the programm smoothly */
    cam_new_frame_arrived = true;
    rear_cam_new_frame_arrived = true;
    cam_condition_variable.notify_one();
    rear_cam_condition_variable.notify_one();



    clear_release();
    RCLCPP_INFO(this->get_logger(), "Stop.");
}

void SensorCam::CamInThread()
{
    double diff_time = 0.0, CycleTime_ = 0.0;
    int cnt = 0;
    const auto wait_duration = std:::chrono::milliseconds(2000);

    while(!imageStatus_ && !rearImageSTatus_) {

        /* Syncronize at startup*/
        unique_lock<mutex> lock(cam_mutex);
        if (cam_condition_variable.wait_for(lock, wait_duration, [this] { return (imageStatus_ && !rearImageStatus_); } )) {
            /* Start done! We are ready to start */
            RCLCPP_INFO(this->get_logger(), "First image arrived.\n");
            break
    }   else {
        /* Timeout - Still waiting... */
            RCLCPP_INFO(this->get_logger(), "Waiting for image.\n");
        if(!isNodeRunning_) {
            return;
        }

    }
}
}

void SensorCam::LoadParams(void)
{

}
cv::Point2f SensorCam::transformPoint(const cv::Point& pt, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    std::vector<cv::Point2f> srcPoints = { cv::Point2f(pt.x, pt.y) }; // 정수형 좌표를 부동소수점으로 변환
    std::vector<cv::Point2f> dstPoints;

    cv::undistortPoints(srcPoints, dstPoints, camera_matrix, dist_coeffs, cv::noArray(), camera_matrix);

    if (dstPoints.size() > 0) {
        return dstPoints[0];
    } else {
        return cv::Point2f(pt.x, pt.y);  // 변환에 실패한 경우 원래 좌표 반환
    }
}

void SensorCam::XavSubCallback(const ros2_msg::msg::Xav2lane::SharedPtr msg)
{
  if(imageStatus_) {
    cur_vel_ = msg->cur_vel;
    distance_ = msg->cur_dist; // for ICRA
    droi_ready_ = true;
  
    get_steer_coef(cur_vel_);
  
    lc_right_flag = msg->lc_right_flag;
    lc_left_flag = msg->lc_left_flag;

    cv::Point2f topLeft = transformPoint(cv::Point(msg->x, msg->y), f_camera_matrix, f_dist_coeffs);
    cv::Point2f bottomRight = transformPoint(cv::Point(msg->x + msg->w, msg->y + msg->h), f_camera_matrix, f_dist_coeffs);

    name_ = msg->name;
    x_ = static_cast<int>(topLeft.x);
    y_ = static_cast<int>(topLeft.y);
    w_ = static_cast<int>(bottomRight.x - topLeft.x);
    h_ = static_cast<int>(bottomRight.y - topLeft.y);
  }

  if(rearImageStatus_) {
    cur_vel_ = msg->cur_vel;

    cv::Point2f topLeft = transformPoint(cv::Point(msg->rx, msg->ry),r_camera_matrix, r_dist_coeffs);
    cv::Point2f bottomRight = transformPoint(cv::Point(msg->rx + msg->rw, msg->ry + msg->rh), r_camera_matrix, r_dist_coeffs);

    r_name_ = msg->r_name;
    rx_ = static_cast<int>(topLeft.x);
    ry_ = static_cast<int>(topLeft.y);
    rw_ = static_cast<int>(bottomRight.x - topLeft.x);
    rh_ = static_cast<int>(bottomRight.y - topLeft.y);
  }
}

void SensorCam::ImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  static cv::Mat prev_img;
//  static double diff_time;
//  static double CycleTime_;
//  static float cnt;
//  struct timeval end_time;

  cv_bridge::CvImagePtr cam_image;
  try{
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception : %s", e.what());
  }

  if(!cam_image->image.empty()) {
//    /* delay time */
//    gettimeofday(&end_time, NULL);
//    imageHeader_ = msg->header;
//    diff_time += ((end_time.tv_sec - imageHeader_.stamp.sec) * 1000.0) + ((end_time.tv_usec - imageHeader_.stamp.nanosec/1000.0) / 1000.0);
//     
//    cnt++;
//
//    CycleTime_ = diff_time / (double)cnt;
//    RCLCPP_INFO(this->get_logger(), "Cycle Time        : %3.3f ms\n", CycleTime_);
//
//    if (cnt > 3000){
//      diff_time = 0.0;
//      cnt = 0;
//    }
//    /* delay time */

    camImageCopy_ = cam_image->image.clone();
    prev_img = camImageCopy_;
    imageStatus_ = true;
    cam_new_frame_arrived = true;
    cam_condition_variable.notify_one();
  }
  else if(!prev_img.empty()) {
    camImageCopy_ = prev_img;
  }
}

void SensorCam::rearImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  Mat frame_;
  cv_bridge::CvImagePtr rear_cam_image;
  try{
    rear_cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception : %s", e.what());
  }

  if(!rear_cam_image->image.empty()) {
    rearImageHeader_ = msg->header;
    rearCamImageCopy_ = rear_cam_image->image.clone();
    frame_ = camImageCopy_;
    rearImageStatus_ = true;

    rear_cam_new_frame_arrived = true;
    rear_cam_condition_variable.notify_one();
  }
}
}