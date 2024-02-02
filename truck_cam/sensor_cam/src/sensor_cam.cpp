#include <include/sensor_cam.hpp>

/* Carla 로 넘기기 */
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
  std::string CarlaCamTopicName;
  int CarlaCamSubQueueSize;
  
  std::string CamPubTopicName;
  int CamPubQueueSize;

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  this->get_parameter_or("subscribers/carlacam_to_cam/topic", CarlaCamSubTopicName, std::string("carlacam2cam_msg"));
  this->get_parameter_or("subscribers/carlacam_to_cam/queue_size", CarlaSubQueueSize, 1);

  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  this->get_parameter_or("publishers/cam_to_lanedetection/topic", CamPubTopicName, std::string("cam2land_msg"));
  this->get_parameter_or("publishers/cam_to_lanedetection/queue_size", CamPubQueueSize, 1);
  
  /************************/
  /* Ros Topic Subscriber */
  /************************/
  CarlaSubscriber_ = this->create_subscription<ros2_msg::msg::CarlaCam2Cam>(CarlaCamSubTopicName, CalraSubQueueSize, std::bind(&SensorCam::CarlaCamSubCallback, this, std::placeholders::_1));

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  CamPublisher_ = this->create_publisher<ros2_msg::msg::Cam2LaneD>(CamPubTopicName, CamPubQueueSize);

  /***************/
  /* View Option */
  /***************/

  /******* recording log *******/    
  gettimeofday(&start_, NULL);

  /******* Camera  calibration *******/

  /*** front cam calibration  ***/
  
  /*** rear cam calibration  ***/

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
    cam_condition_variable.notify_one(); // 하나의 스레드를 깨운다? 이게뭐임?

    clear_release();
    RCLCPP_INFO(this->get_logger(), "Camera Stop.");
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

void SensorCam::CalraCamSubCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  static cv::Mat prev_img;
  cv_bridge::CvImagePtr cam_image;
  try{
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception : %s", e.what());
  }

  if(!cam_image->image.empty()) {

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
}