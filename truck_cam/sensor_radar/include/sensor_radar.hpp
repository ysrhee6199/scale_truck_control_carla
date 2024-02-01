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
// Eigen
#include <Eigen/Core>
#include <sensor_msgs/msgs.pointcloud2.hpp>
#include <std_msgs/msg/header.hpp>
//ROS2
#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/radar2laned.hpp"


#define _GUN_SOURCE

using namespace std;
using namespace Eigen;

namespace Sensor {
    class SensorRadar : public rclcpp::Node
{
public:
    SensorRadar();
    ~SensorRadar();
private:
    void LoadParams(void);

    //Subscriber
    rclcpp::Subscrioption<sensor_msgs::msg::RadarPointCloud>::SharedPtr RadarSubscriber_;
    
    //Callback Func
    void RadarsubCallback(const sensor_msgs::msg::RadarPointCloud::SharedPtr msg);

    bool veiwRadar_;
    int waitkeyDelay_;
    bool isNodeRunning_;

    //Radar PointCloud
    bool radarStatus_ = false

    





    /* Callback Synchronization */






}

}

