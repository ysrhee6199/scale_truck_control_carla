#include "lane_keeping.hpp"


namespace Control {

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

void LaneKeeping::get_steer_coef(float vel) {
    float value;
    if (vel > 1.2f) {
        value = 1.2f;
    }
    else {
        value = vel;
    }

    if (value < 0.65f) {
        K1_ = K2_ = K_;
    }
    else {
        K1_ = (a_[0] * pow(value, 4)) + (a_[1] * pow(value, 3)) + (a_[2] * pow(value, 2)) + (a_[3] * value) + a_[4]
        K2_ = (b_[0] * pow(value, 4)) + (b_[1] * pow(value, 3)) + (b_[2] * pow(value, 2)) + (b_[3] * value) + b_[4]
    }

}

void LaneKeeping::controlSteer() {
  Mat l_fit(left_coef_), r_fit(right_coef_), c_fit(center_coef_), e_fit(extra_coef_), e2_fit(extra2_coef_), c2_fit(center2_coef_), c3_fit(center3_coef_);
  float car_position = width_ / 2;
  float l1 = 0.0f, l2 = 0.0f, l3 = 0.0f;
  float i = ((float)height_) * eL_height_;  
  float j = ((float)height_) * trust_height_;
  float k = ((float)height_) * e1_height_;
  float temp_diff = 10.1f;
  static int wroi_cnt_ = 0;

  lane_coef_.coef.resize(3);
  if (!l_fit.empty() && !r_fit.empty()) {
    lane_coef_.coef[0].a = l_fit.at<float>(2, 0);
    lane_coef_.coef[0].b = l_fit.at<float>(1, 0);
    lane_coef_.coef[0].c = l_fit.at<float>(0, 0);

    lane_coef_.coef[1].a = r_fit.at<float>(2, 0);
    lane_coef_.coef[1].b = r_fit.at<float>(1, 0);
    lane_coef_.coef[1].c = r_fit.at<float>(0, 0);

    lane_coef_.coef[2].a = c_fit.at<float>(2, 0);
    lane_coef_.coef[2].b = c_fit.at<float>(1, 0);
    lane_coef_.coef[2].c = c_fit.at<float>(0, 0);

    l1 =  j - i;
    l2 = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(j, 2)) + (lane_coef_.coef[2].b * j) + lane_coef_.coef[2].c);

    e_values_[0] = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - car_position;  //eL
    e_values_[1] = e_values_[0] - (lp_ * (l2 / l1));  //trust_e1
    e_values_[2] = ((lane_coef_.coef[2].a * pow(k, 2)) + (lane_coef_.coef[2].b * k) + lane_coef_.coef[2].c) - car_position;  //e1
    SteerAngle_ = ((-1.0f * K1_) * e_values_[1]) + ((-1.0f * K2_) * e_values_[0]);
  }

  /*********************/
  /* right lane change */
  /*********************/
  if ((!center2_coef_.empty()) && lc_right_flag_ && wroi_flag_) {
    lane_coef_.coef[0].a = r_fit.at<float>(2, 0);
    lane_coef_.coef[0].b = r_fit.at<float>(1, 0);
    lane_coef_.coef[0].c = r_fit.at<float>(0, 0);

    lane_coef_.coef[1].a = e_fit.at<float>(2, 0);
    lane_coef_.coef[1].b = e_fit.at<float>(1, 0);
    lane_coef_.coef[1].c = e_fit.at<float>(0, 0);

    lane_coef_.coef[2].a = c2_fit.at<float>(2, 0);
    lane_coef_.coef[2].b = c2_fit.at<float>(1, 0);
    lane_coef_.coef[2].c = c2_fit.at<float>(0, 0);

    mark_ = 1;
    tk::spline cspline_eq_ = cspline();

    l3 = (float)cspline_eq_(i) - (float)cspline_eq_(j);
    e_values_[0] = (float)cspline_eq_(i) - car_position;  //eL
    e_values_[1] = e_values_[0] - (lp_ * (l3 / l1));  //trust_e1
    e_values_[2] = (float)cspline_eq_(k)- car_position;  //e1
    SteerAngle2_ = ((-1.0f * K3_) * e_values_[1]) + ((-1.0f * K4_) * e_values_[0]);

    /* cspline follow -> center follow  */
    temp_diff = ((lane_coef_.coef[2].a * pow(height_, 2)) + (lane_coef_.coef[2].b * height_) + lane_coef_.coef[2].c) - (float)cspline_eq_(height_); 
    temp_diff = abs(temp_diff);

    if (center_select_ == 2 && ((0 <= temp_diff) && (temp_diff <= 10))) {
      wroi_cnt_ += 1;
      if (wroi_cnt_ >= 50)  {
        wroi_flag_ = false;
        wroi_cnt_ = 0;
      }
    }
//    if (center_select_ == 2 && ((0 <= temp_diff) && (temp_diff <= 10))) {
//      lc_center_follow_ = true; 
//      l1 =  j - i;
//      l2 = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(j, 2)) + (lane_coef_.coef[2].b * j) + lane_coef_.coef[2].c);
//  
//      e_values_[0] = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - car_position;  //eL
//      e_values_[1] = e_values_[0] - (lp_ * (l2 / l1));  //trust_e1
//      SteerAngle2_ = ((-1.0f * K1_) * e_values_[1]) + ((-1.0f * K2_) * e_values_[0]);
//    }
  /* cspline follow -> center follow  */
  }
  /********************/
  /* left lane change */
  /********************/
  else if ((!center3_coef_.empty()) && lc_left_flag_ && wroi_flag_) { 
    lane_coef_.coef[0].a = e2_fit.at<float>(2, 0);
    lane_coef_.coef[0].b = e2_fit.at<float>(1, 0);
    lane_coef_.coef[0].c = e2_fit.at<float>(0, 0);

    lane_coef_.coef[1].a = l_fit.at<float>(2, 0);
    lane_coef_.coef[1].b = l_fit.at<float>(1, 0);
    lane_coef_.coef[1].c = l_fit.at<float>(0, 0);

    lane_coef_.coef[2].a = c3_fit.at<float>(2, 0);
    lane_coef_.coef[2].b = c3_fit.at<float>(1, 0);
    lane_coef_.coef[2].c = c3_fit.at<float>(0, 0);

    mark_ = 2;
    tk::spline cspline_eq_ = cspline();

    l3 = (float)cspline_eq_(i) - (float)cspline_eq_(j);
    e_values_[0] = (float)cspline_eq_(i) - car_position;  //eL
    e_values_[1] = e_values_[0] - (lp_ * (l3 / l1));  //trust_e1
    e_values_[2] = (float)cspline_eq_(k)- car_position;  //e1
    SteerAngle2_ = ((-1.0f * K3_) * e_values_[1]) + ((-1.0f * K4_) * e_values_[0]);

    temp_diff = ((lane_coef_.coef[2].a * pow(height_, 2)) + (lane_coef_.coef[2].b * height_) + lane_coef_.coef[2].c) - (float)cspline_eq_(height_); 
    temp_diff = abs(temp_diff);

    if (center_select_ == 3 && ((0 <= temp_diff) && (temp_diff <= 10))) {
      wroi_cnt_ += 1;
      if (wroi_cnt_ >= 50)  {
        wroi_flag_ = false;
        wroi_cnt_ = 0;
      }
    }
    /* cspline follow -> center follow  */
//    if (center_select_ == 3 && ((0 <= temp_diff) && (temp_diff <= 10))) {
//      lc_center_follow_ = true; 
//      l1 =  j - i;
//      l2 = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(j, 2)) + (lane_coef_.coef[2].b * j) + lane_coef_.coef[2].c);
//  
//      e_values_[0] = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - car_position;  //eL
//      e_values_[1] = e_values_[0] - (lp_ * (l2 / l1));  //trust_e1
//      SteerAngle2_ = ((-1.0f * K1_) * e_values_[1]) + ((-1.0f * K2_) * e_values_[0]);
//    }
    /* cspline follow -> center follow  */
  }
}

tk::spline LaneKeeping::cspline() {
    float tY1_ = ((float)height_) * 0;
    float tY2_ = ((float)height_) * 0.1;
    float tY3_ = ((float)height_) * 0;
    float tY4_ = ((float)height_) * 0.1;
    float tY5_ = ((float)height_) * e1_height_;

    std::vector<double> X;
    std::vector<double> Y;
    tk::spline cspline_eq;
    /*****************/
    /* lc_right_coef */
    /*****************/
    if (mark_ == 1 && !right_coef_.empty() && !center2_coef_.empty()) {
        int tX1_ = (int)((right_coef_.at<float>(2, 0) * pow(tY1_, 2)) + (right_coef_.at<float>(1, 0) * tY1_) + right_coef_.at<float>(0, 0));
        int tX2_ = (int)((right_coef_.at<float>(2, 0) * pow(tY2_, 2)) + (right_coef_.at<float>(1, 0) * tY2_) + right_coef_.at<float>(0, 0));
        int tX3_ = (int)((center2_coef_.at<float>(2, 0) * pow(tY3_, 2)) + (center2_coef_.at<float>(1, 0) * tY3_) + center2_coef_.at<float>(0, 0));
        int tX4_ = (int)((center2_coef_.at<float>(2, 0) * pow(tY4_, 2)) + (center2_coef_.at<float>(1, 0) * tY4_) + center2_coef_.at<float>(0, 0));

        if (center_select_ == 1) {
            X = {(double)tX1_, (double)tX2_, (double)(width/2)};

            Y = {(double)tY1_, (double)tY2_, (double)height_};
        }
        else if (center_select_ == 2) {
            X = {(double)tX3_, (double)tX4_, (double)(width_/2)};

            Y = {(double)tY3_, (double)tY4_, (double)height_};
        }
        else {
            X = {(double)tX3_, (double)tX4_, (double)(width_/2)};

            Y = {(double)tY3_, (double)tY4_, (double)height_};
        }

        tk::spline s(Y, X, tk::spline::cspline);
        cspline_eq = s;
    }
    else if (mark_ == 2 && !left_coef_.empty() && !center3_coef_.empty()) {
        int tX1_ = (int)((left_coef_.at<float>(2, 0) * pow(tY1_, 2)) + (left_coef_.at<float>(1, 0) * tY1_) + left_coef_.at<float>(0, 0));
        int tX2_ = (int)((left_coef_.at<float>(2, 0) * pow(tY2_, 2)) + (left_coef_.at<flaot>(1, 0) * tY2_) + left_coef_.at<float>(0, 0));
        int tX3_ = (int)((center3_coef_.at<float>(2, 0) * pow(tY3_, 2)) + (center3_coef_.at<float>(1, 0) * tY3_) + center3_coef_.at<float>(0, 0));
        int tX4_ = (int)((center3_coef_.at<float>(2, 0) * pow(tY4_, 2)) + (center3_coef_.at<float>(1, 0) * tY4_) + center3_coef_.at<float>(0, 0));

        if (center_select_ == 1) {
            X = {(double)tX1_, (double)tX2_, (double)(width_/2)};

            Y = {(double)tY1_, (double)tY2_, (double)height_};
        }
        else if (center_select_ == 2) {
            X = {(double)tX3_, (double)tX4_, (double)(width_/2)};

            Y = {(double)tY3_, (double)tY4_, (double)height_};
        }
        else {
            X = {(double)tX3_, (double)tX4_, (double)(width_/2)};

            Y = {(double)tY3_, (double)tY4_, (double)height_};
        }
        tk::spline s(Y, X, tk::spline::cspline);
        cspline_eq = s;
    }

    return cspline_eq;

}



}