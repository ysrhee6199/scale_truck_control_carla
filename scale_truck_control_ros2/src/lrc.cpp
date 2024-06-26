#include "lrc/lrc.hpp"

using namespace std::chrono_literals;

namespace LocalResiliencyCoordinator{

LocalRC::LocalRC(void) 
  : Node("LRC", rclcpp::NodeOptions()
                      .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true))
{
  init();

}

LocalRC::~LocalRC(){
  isNodeRunning_ = false; 
  udpThread_.join();
}

void LocalRC::init(){

  isNodeRunning_ = true;

  std::string XavSubTopicName;
  int XavSubQueueSize;
  std::string OcrSubTopicName;
  int OcrSubQueueSize;
  std::string LVSubTopicName;
  int LVSubQueueSize;
  std::string XavPubTopicName;
  int XavPubQueueSize;
  std::string OcrPubTopicName;
  int OcrPubQueueSize;
  std::string FVPubTopicName;
  int FVPubQueueSize;

  this->get_parameter_or("LrcParams/lrc_index", index_, 10);
  this->get_parameter_or("LrcParams/lrc_log_path", log_path_, std::string("/home/jetson/catkin_ws/logfiles/"));
  this->get_parameter_or("LrcParams/enable_console_output", EnableConsoleOutput_, true);
  /******************************/
  /* ROS Topic Subscribe Option */
  /******************************/
  this->get_parameter_or("LrcSub/xavier_to_lrc/topic", XavSubTopicName, std::string("xav2lrc_msg"));
  this->get_parameter_or("LrcSub/xavier_to_lrc/queue_size", XavSubQueueSize, 1);
  this->get_parameter_or("LrcSub/ocr_to_lrc/topic", OcrSubTopicName, std::string("ocr2lrc_msg"));
  this->get_parameter_or("LrcSub/ocr_to_lrc/queue_size", OcrSubQueueSize, 1);
  this->get_parameter_or("LrcSub/lv_to_fv/topic", LVSubTopicName, std::string("/lv2fv_msg"));
  this->get_parameter_or("LrcSub/lv_to_fv/queue_size", LVSubQueueSize, 1);

  /******************************/
  /* ROS Topic Publish Option */
  /******************************/
  this->get_parameter_or("LrcPub/lrc_to_xavier/topic", XavPubTopicName, std::string("lrc2xav_msg"));
  this->get_parameter_or("LrcPub/lrc_to_xavier/queue_size", XavPubQueueSize, 1);
  this->get_parameter_or("LrcPub/lrc_to_ocr/topic", OcrPubTopicName, std::string("lrc2ocr_msg"));
  this->get_parameter_or("LrcPub/lrc_to_ocr/queue_size", OcrPubQueueSize, 1);
  this->get_parameter_or("LrcPub/lv_to_fv/topic", FVPubTopicName, std::string("/lv2fv_msg"));
  this->get_parameter_or("LrcPub/lv_to_fv/queue_size", FVPubQueueSize, 1);

  /************************/
  /* ROS Topic Subscriber */
  /************************/
  OcrSubscriber_ = this->create_subscription<ros2_msg::msg::Ocr2lrc>(OcrSubTopicName, OcrSubQueueSize, std::bind(&LocalRC::OcrCallback, this, std::placeholders::_1));

  XavSubscriber_ = this->create_subscription<ros2_msg::msg::Xav2lrc>(XavSubTopicName, XavSubQueueSize, std::bind(&LocalRC::XavCallback, this, std::placeholders::_1));

  if (index_ == 11){
    LVSubscriber_ = this->create_subscription<ros2_msg::msg::Lrc2xav>(LVSubTopicName, LVSubQueueSize, std::bind(&LocalRC::LVCallback, this, std::placeholders::_1));
  }
  else if (index_ == 12) {
    LVSubscriber_ = this->create_subscription<ros2_msg::msg::Lrc2xav>(std::string("/fv12fv2_msg"), LVSubQueueSize, std::bind(&LocalRC::LVCallback, this, std::placeholders::_1));
  }

  /************************/
  /* ROS Topic Publisher */
  /************************/
  XavPublisher_ = this->create_publisher<ros2_msg::msg::Lrc2xav>(XavPubTopicName, XavPubQueueSize);  
  OcrPublisher_ = this->create_publisher<ros2_msg::msg::Lrc2ocr>(OcrPubTopicName, OcrPubQueueSize);
  if (index_ == 10) {
    FVPublisher_ = this->create_publisher<ros2_msg::msg::Lrc2xav>(FVPubTopicName, FVPubQueueSize);
  }
  else if(index_ == 11) {
    FVPublisher_ = this->create_publisher<ros2_msg::msg::Lrc2xav>(std::string("/fv12fv2_msg"), FVPubQueueSize);
  }

  /*********************/
  /* spin & udp thread */
  /*********************/
  lrcThread_ = std::thread(&LocalRC::communicate, this);
  if (index_ == 10 || index_ == 11){
    udpThread_ = std::thread(&LocalRC::radio, this);
  }

}

bool LocalRC::isNodeRunning(){
  return isNodeRunning_;
}

/****************/
/* FVs Publish  */
/****************/
void LocalRC::radio()
{
  ros2_msg::msg::Lrc2xav lv_data_;
  while(isNodeRunning()){
    {
      std::scoped_lock lock(data_mutex_);
      lv_data_.tar_vel = cur_vel_;  // current speed
      lv_data_.tar_dist = tar_dist_; // contstant time_gap
      lv_data_.emergency_flag = emergency_flag_;
    }

    FVPublisher_->publish(lv_data_); 
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void LocalRC::rosPub(){
  ros2_msg::msg::Lrc2xav xav;
  ros2_msg::msg::Lrc2ocr ocr;
  { 
    std::scoped_lock lock(data_mutex_);
    xav.cur_vel = cur_vel_;
    xav.tar_vel = tar_vel_; // xav에 목표 속도,간격 보내는 이유: 
    xav.tar_dist = tar_dist_; // FV는 LV LRC에서 전달 받기 때문
    ocr.index = index_;
    ocr.steer_angle = angle_degree_;
    ocr.cur_dist = cur_dist_;
    ocr.tar_dist = tar_dist_;
    ocr.tar_vel = tar_vel_;
    ocr.est_vel = est_vel_;
    ocr.emergency_flag = emergency_flag_;
  }
  XavPublisher_->publish(xav);
  OcrPublisher_->publish(ocr);
}

void LocalRC::recordData(struct timeval *startTime){
  struct timeval currentTime;
  char file_name[] = "LocalRC_log00.csv";
  static char file[128] = {0x00, };
  char buf[256] = {0x00,};
  static bool flag = false;
  std::ifstream read_file;
  std::ofstream write_file;
  if(!flag){
    for(int i = 0; i < 100; i++){
      file_name[7] = i/10 + '0';  //ASCII
      file_name[8] = i%10 + '0';
      sprintf(file, "%s%s", log_path_.c_str(), file_name);
      read_file.open(file);
      if(read_file.fail()){  //Check if the file exists
        read_file.close();
        write_file.open(file);
        break;
      }
      read_file.close();
    }
    write_file << "Time,Tar_dist,Cur_dist,Tar_vel,Ref_vel,Cur_vel,Lrc_mode" << std::endl; //seconds
    flag = true;
  }
  else{
    std::scoped_lock lock(data_mutex_);
    gettimeofday(&currentTime, NULL);
    time_ = ((currentTime.tv_sec - startTime->tv_sec)) + ((currentTime.tv_usec - startTime->tv_usec)/1000000.0);
    //sprintf(buf, "%.10e,%.3f,%.3f,%.3f,%.3f,%.3f,%d", time_, tar_dist_, cur_dist_, tar_vel_, ref_vel_, cur_vel_);
    write_file.open(file, std::ios::out | std::ios::app);
    write_file << buf << std::endl;
  }
  write_file.close();
}

void LocalRC::printStatus(){
  if (EnableConsoleOutput_){
    printf("\033[2J");
    printf("\033[1;1H");
    printf("LrcParams/lrc_index :%d\n", index_);
    printf("Target Velocity:\t%.3f\n", tar_vel_);
    printf("Current Velocity:\t%.3f\n", cur_vel_);
    printf("Target Distance:\t%.3f\n", tar_dist_);
    printf("Current Distance:\t%.3f\n", cur_dist_);
    printf("ocr_vel:\t%.3f\n", ref_vel_);
    printf("sat_vel:\t%.3f\n", sat_vel_);
//    printf("Tar|Cur Vel     : %3.3f | %3.3f m/s\n", tar_vel_, cur_vel_);
//    printf("Tar|Cur Dist    : %3.3f | %3.3f m\n", tar_dist_, cur_dist_);

  }
}


void LocalRC::communicate(){
  while(rclcpp::ok()){
//    struct timeval start_time, end_time;
//    gettimeofday(&start_time, NULL);
//    static float cnt;
//    static double diff_time, CycleTime_;

    rosPub();
    printStatus();

    //recordData(&startTime);

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    if(!isNodeRunning()){
      rclcpp::shutdown();
      break;
    }

//    gettimeofday(&end_time, NULL);
//    diff_time += ((end_time.tv_sec - start_time.tv_sec) * 1000.0) + ((end_time.tv_usec - start_time.tv_usec) / 1000.0);
//    cnt++;
//
//    CycleTime_ = diff_time / (double)cnt;
//    RCLCPP_INFO(this->get_logger(), "delay Time        : %3.3f ms\n", CycleTime_);
//
//    if (cnt > 3000){
//      diff_time = 0.0;
//      cnt = 0;
//    }
  }
}

void LocalRC::XavCallback(const ros2_msg::msg::Xav2lrc::SharedPtr msg)
{
  std::scoped_lock lock(data_mutex_);
  angle_degree_ = msg->steer_angle;
  cur_dist_ = msg->cur_dist;
  
  if(index_ == 10){  //only LV LRC
    tar_vel_ = msg->tar_vel;
    tar_dist_ = msg->tar_dist;
    emergency_flag_ = msg->emergency_flag;
  }

//  /* delay time */
//  static double stamp_time_sec, stamp_time_usec;
//  static double delay_, diff_time;
//  static float cnt_;
//  struct timeval end_time;
//
//  stamp_time_sec = msg->stamp_sec;
//  stamp_time_usec = msg->stamp_usec;
//
//  gettimeofday(&end_time, NULL);
//  diff_time += ((end_time.tv_sec - stamp_time_sec) * 1000.0) + ((end_time.tv_usec - stamp_time_usec) / 1000.0);
//
//  cnt_++;
//
//  delay_ = diff_time / (double)cnt_;
//  RCLCPP_INFO(this->get_logger(), "delay Time        : %3.3f ms\n", delay_);
//
//  if (cnt_ > 3000){
//    diff_time = 0.0;
//    cnt_ = 0;
//  }
//  /* delay time */

}

void LocalRC::OcrCallback(const ros2_msg::msg::Ocr2lrc::SharedPtr msg)   
{
  std::scoped_lock lock(data_mutex_);
  ref_vel_ = msg->ref_vel;
  cur_vel_ = msg->cur_vel;
  sat_vel_ = msg->u_k;  //saturated velocity
}

/***************/
/* FVs from LV */
/***************/
void LocalRC::LVCallback(const ros2_msg::msg::Lrc2xav::SharedPtr msg)
{
  std::scoped_lock lock(data_mutex_);
  tar_vel_ = msg->tar_vel;
  tar_dist_ = msg->tar_dist;
  emergency_flag_ = msg->emergency_flag;
}



} /* namespace scale_truck_control */

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalResiliencyCoordinator::LocalRC>());
    rclcpp::shutdown();
    return 0;
}

