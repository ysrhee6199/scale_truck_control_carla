#include <include/sensor_radar.hpp>

namespace Sensor {

SensorRadar::SensorRadar()
        :Node("SensorRadar", rclcpp::NodeOptions()
                            .allow_undeclared_parameters(true)
                            .automatically_declare_parameters_from_overrides(true))
{
    /**************/
    /* ROS2 Topic */
    /**************/
    
    std::string RadarSubTopicName;
    int RadarSubQueueSize;

    /******************************/
    /* Ros Topic Subscribe Option (Yet)*/
    /******************************/
    this->get_parameter_or("subscribers/radar_to_lane/topic", RadarSubTopicName, std::string(""));
    this->get_parameter_or("subscribers/radar_to_lane/queue_size", RadarSubQueueSize, 1);
    
    /****************************/
    /* Ros Topic Publish Option */
    /****************************/
  
    /************************/
    /* Ros Topic Subscriber */
    /************************/

    RadarSubscriber_ = this->create_subscription<sensor_msgs::msg::Radar>(RadarSubTopicName, RadarSubQueueSize, std::bind(&SensorRadar::RadarsubCallback, this, std::placeholders::_1));
    


    isNodeRunning_ = true;
}


SensorRadar::~SensorRadar(void)
{
    isNodeRunning_ = false;

    /* Unblock the other thread to shutdown the programm smoothly */
    // notify_one ?
    radar_new_arrived = true;
    radar_condition_variable.notify_one();

    claer_relaese();
    RCLCPP_INFO(this->get_logger(), "Radar Stop.")
}

void SensorRadar::RadarsubCallback(const sensor_msgs::msg::RadarPointCloud::SharedPtr msg)
{
    // Eigen3 //
}
}