#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>

// Carla //
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <carla/sensor/data/RadarMeasurement.h>
#include "sensor_msgs/msg/point_cloud2.hpp"


namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
using namespace std::chrono_literals;
using namespace std::string_literals;



namespace Sensor {
    class SensorRadar : public rclcpp::Node
{
public:
    SensorRadar();
    ~SensorRadar(){
        radar->Destroy();
    }
private:
    boost::shared_ptr<carla::client::BlueprintLibrary> blueprint_library = nullptr;
    boost::shared_ptr<carla::client::Actor> actor = nullptr;
    carla::client::World& world_;
    //Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr RadarPublisher_;
    
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

