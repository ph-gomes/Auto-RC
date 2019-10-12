/*
 * RoadLineProcessing.hpp
 *
 *  Created on: Fev 25, 2018
 *      Author: giovani
 */

#ifndef SRC_ROADLINEPROCESSING_HPP_
#define SRC_ROADLINEPROCESSING_HPP_

#include <algorithm>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"

#include <vector>

#include <string>
#include <cstdarg>
#include <memory>

#include "../controller/controller.h"
#include "../lsd_1.6/lsd.h"
#include "../polyfit/polyfit.hpp"

#include <fstream>

double peso1 = 0.0;
double peso2 = 0.0;

/**
class to receive the handle of ros with the image
**/
class RoadLineProcessing
{

    int width_image;
    int heigh_image;

    unsigned char *search_mask;
    double *image;

    cv::Mat_<float> *desired_line;  // [x,y] (pixels), [m] (radians), [b] (pixels)
    cv::Mat_<float> *error_control; // error_x and error_theta
    cv::Mat_<float> *offset;        //offset map

    cv::Mat o_img;
    cv::Mat lambda;
    cv::Mat r_lambda;

    std::vector<double> previous_poly;

    double p_y;

    bool process_stage;
    bool initializer;

    double ex_ant;
    double ex_sum;

    double dt_ctr;
    double t_ant;
    double t_res;

    double sigma, sigma_x, sigma_xx;
    double kp, ki, kd;

    double dt_v;
    int ctr_sp;

    double weight;
    int GB_Size;
    double Hor_line_thresh;

public:
    double v_set;
    double steering;
    double velocity;

    RoadLineProcessing(ros::NodeHandle nh);
    ~RoadLineProcessing();

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

private:
    void initialConfig(int X, int Y);

    void line_detection(cv::Mat img);

    void reset(cv::Point2f *start, cv::Point2f *end);
    void speed_control(int gs);
    void init_temporal_line_filtering(cv::Point2f *start, cv::Point2f *end, std::vector<double> *p_coeffs, std::vector<double> *coeffs);
    void temporal_line_filtering(cv::Point2f *start, cv::Point2f *end, std::vector<double> *p_coeffs, std::vector<double> *coeffs, double &weight);
    double visual_servoing_control(cv::Point2f observed_pt, float observed_angle);
};

#endif /* SRC_ROADLINEPROCESSING_HPP_ */
