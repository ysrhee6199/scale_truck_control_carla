#include "lane_keeping.hpp"


namespace LaneKeeping {

LaneKeeping::LaneKeeping()


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

    /* Lateral Control coefficient */
    this->get_parameter_or("params/K", K_, 0.15f);
    this->get_parameter_or("params/K3", K3_, 0.15f);
    this->get_parameter_or("params/K4", K4_, 0.15f);

    this->get_parameter_or("params/a/a", a_[0], 0.);
    this->get_parameter_or("params/a/b", a_[1], -0.37169);
    this->get_parameter_or("params/a/c", a_[2], 1.2602);
    this->get_parameter_or("params/a/d", a_[3], -1.5161);
    this->get_parameter_or("params/a/e", a_[4], 0.70696);

    this->get_parameter_or("params/b/a", b_[0], 0.);
    this->get_parameter_or("params/b/b", b_[1], -1.7536);
    this->get_parameter_or("params/b/c", b_[2], 5.0931);
    this->get_parameter_or("params/b/d", b_[3], -4.9047);
    this->get_parameter_or("params/b/e", b_[4], 1.6722);

    this->get_parameter_or("params/a2/a", a2_[0], 0.5);
    this->get_parameter_or("params/a2/b", a2_[1], -0.95);
    this->get_parameter_or("params/a2/c", a2_[2], 0.52);

    this->get_parameter_or("params/b2/a", b2_[0], -0.0875);
    this->get_parameter_or("params/b2/b", b2_[1], 0.0575);
    this->get_parameter_or("params/b2/c", b2_[2], 0.17);
}

LaneKeeping::~LaneKeeping(void)
{

}

void LaneKeeping::get_steer_coef(float vel){

}

void LaneKeeping::controlSteer() {

}

tk::spline LaneKeeping::cspline() {

}



}