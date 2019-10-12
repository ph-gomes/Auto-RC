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

struct PointXYZ{
	double x, y, z;
};

typedef std::vector<float> VectorXf;

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}
template <typename T>
T clip(const T &n, const T &lower, const T &upper)
{
    return std::max(lower, std::min(n, upper));
}

template <typename T>
int round_int(T r)
{
    return (r > 0.0) ? (r + 0.5) : (r - 0.5);
}

// ==========================================================================================================================
//  Funcoes auxiliares para realizacao dos testes com as imagens fornecidas...
// ==========================================================================================================================

std::string format(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    #ifndef _MSC_VER
        size_t size = std::snprintf( nullptr, 0, format, args) + 1; // Extra space for '\0'
        std::unique_ptr<char[]> buf( new char[ size ] ); 
        std::vsnprintf( buf.get(), size, format, args);
        return std::string(buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
    #else
        int size = _vscprintf(format, args);
        std::string result(++size, 0);
        vsnprintf_s((char*)result.data(), size, _TRUNCATE, format, args);
        return result;
    #endif
    va_end(args);
}

std::string ZeroPadNumber(int num, int pad)
{
    std::ostringstream ss;
    ss << std::setw(pad) << std::setfill('0') << num;
    return ss.str();
}

bool grab(const char *Prefix, cv::Mat *Img, int img_count = 0)
{

    std::string file = Prefix + std::string("frame") + ZeroPadNumber(img_count, 4) + std::string(".png");
    std::cout << "GRAB FILE: " << file << std::endl;
    *Img = cv::imread(file.c_str());

    std::cout << "Width : " << Img->cols << std::endl
              << "Height: " << Img->rows << std::endl;

    CV_Assert(!Img->empty());

    return true;
}
// ==========================================================================================================================

/**

class to receive the handle of ros with the image
**/
class RoadLineProcessing
{

    int width_image;
    int heigh_image;
    int roiX;
    int roiY;

    bool process_stage;

    unsigned char *search_mask;
    double *image;

    double sigma_m, sigma_b;

    cv::Mat_<float> *mb_ant;

    // Eigen::VectorXf previous_coeffr, previous_coeffl; 
    VectorXf previous_coeffr, previous_coeffl; 

    cv::Mat_<float> *desired_line; // [x,y] (pixels), [m] (radians), [b] (pixels)
	cv::Mat_<float> *error_control; // error_x and error_theta
    cv::Mat_<float> *offset; //offset map

    double ex_ant;
    double ex_sum;
    double t_ant;
    double kp, ki, kd;

    double v_set, dt_v;
    bool ctr_sp;
    
    double dt_lAz;
    double dt_lAm;

    double th_az;
    double th_am;

  public:
    VectorXf previous_Rline;
    double steering;
    double velocity;

    /**
    @brief constructor of class with initi with default parameters
    @TODO: use rosparam to configure the parameters
    **/
    RoadLineProcessing(ros::NodeHandle nh);
    ~RoadLineProcessing();

    /**
    @brief
    ROS callback that return the image from the topic
    @param msg sensor_msgs::Image

    **/
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    void grab(cv::Mat image);

  private:
    void imgConfig(int X, int Y);

    void invert_point(cv::Point2f *start, cv::Point2f *end);
    VectorXf RightLineFitting(std::map<int, double>, double *, cv::Mat image);

    void line_detection(cv::Mat img);
    void lineExtraction(const sensor_msgs::ImageConstPtr &msg);

    void temporal_line_filtering(cv::Point2f *start, cv::Point2f *end, VectorXf *line, double weight);
    void init_temporal_line_filtering(cv::Point2f *start, cv::Point2f *end, VectorXf *line);

    double visual_servoing_control(cv::Point2f statePt1, cv::Point2f statePt2);

    void speed_control(bool gs);
};

#endif /* SRC_ROADLINEPROCESSING_HPP_ */
