#include "lane_detect.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<Control::LaneKeeping>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}