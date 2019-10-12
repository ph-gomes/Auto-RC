/*
 * RoadLineProcessing.cpp
 *
 *  Created on: FEV 24, 2018
 *      Author: giovani
 */

#include <stdio.h>

#include "Road_Line_Processing.hpp"

#define DEBUG

/**
@brief constructor of class with initi with default parameters
@TODO: use rosparam to configure the parameters
**/
RoadLineProcessing::RoadLineProcessing(ros::NodeHandle nh)
{

	process_stage = false;

	//Poderar a filtragem temporal
	sigma_m = 0.65;
	sigma_b = 0.65;

	previous_Rline.resize(4);

	// Vector Information: [0]:x, [1]:y, [2]:m, [3]:b
	previous_Rline[1] = 225.0;

	// cv::Point2f start(521.0, 91.0), end(620.0, 200.0);
	cv::Point2f start(320 / 2, 0), end(320 / 2, 128);

	float m = (double)(end.x - start.x) / (end.y - start.y);
	float b = (double)start.x - m * start.y;

	desired_line = new cv::Mat_<float>(5, 1);
	mb_ant = new cv::Mat_<float>(2, 1);
	error_control = new cv::Mat_<float>(2, 1);

	offset = new cv::Mat_<float>(2, 1);

	float desired_y = 128.0;			 //215.827;
	float desired_x = m * desired_y + b; //round_int(m*desired_y + b);

	(*desired_line)(0) = desired_x;
	(*desired_line)(1) = desired_y;
	(*desired_line)(2) = m;
	(*desired_line)(3) = b;

	(*mb_ant)(0) = m;
	(*mb_ant)(1) = b;

	(*offset)(0) = (*mb_ant)(1) + 31 * sqrt(pow((*mb_ant)(0), 2) + 1);
	(*offset)(1) = (*mb_ant)(1) - 31 * sqrt(pow((*mb_ant)(0), 2) + 1);

	ex_ant = 0;
	ex_sum = 0;

	dt_lAz = ros::Time::now().toSec();
	dt_lAm = ros::Time::now().toSec();
	t_ant = ros::Time::now().toSec();

	th_am = 2;
	th_az = 5;

	v_set = 1.7;
	velocity = 0.0;
	ctr_sp = false;
	dt_v = 0.0;

	nh.param("k_p", kp, 0.0015);
	nh.param("k_i", ki, 0.00);
	nh.param("k_d", kd, 0.00);

	std::cout << "ki: " << ki << " kd: " << kd << " kp: " << kp << std::endl;

#ifdef DEBUG
	cv::namedWindow("view");
	cv::startWindowThread();
#endif
};

RoadLineProcessing::~RoadLineProcessing()
{
	free((void *)image);
	delete (desired_line);
	delete (mb_ant);

#ifdef DEBUG
	cv::destroyWindow("view");
#endif
}

void RoadLineProcessing::imgConfig(int X, int Y)
{
	width_image = X;
	heigh_image = Y;

	image = (double *)malloc((size_t)(X * Y) * sizeof(double));

	process_stage = true;
}

VectorXf RoadLineProcessing::RightLineFitting(std::map<int, double> used_lines, double *out, cv::Mat image)
{
	int dim = 7;
	double mod, mod_p = 0;
	VectorXf coeff_refinedr(6);
	PointXYZ start, end, start_b, end_b;

	std::map<int, double>::iterator it = used_lines.begin();

	if (it != used_lines.end())
	{
		for (; it != used_lines.end(); ++it)
		{
			int index = it->first - 1;

			start.x = (float)out[index * dim + 0];
			start.y = (float)out[index * dim + 1];
			start.z = 0.0;
			end.x = (float)out[index * dim + 2];
			end.y = (float)out[index * dim + 3];
			end.z = 0.0;

			mod = sqrt((start.x - end.x) * (start.x - end.x) +
					   (start.y - end.y) * (start.y - end.y) +
					   (start.z - end.z) * (start.z - end.z));

			if (mod > mod_p)
			{
				mod_p = mod;
				start_b = start;
				end_b = end;
			}
		}
	}
	else
		return previous_coeffr;

	coeff_refinedr[0] = start_b.x;
	coeff_refinedr[1] = start_b.y;
	coeff_refinedr[2] = start_b.z;

	coeff_refinedr[3] = end_b.x;
	coeff_refinedr[4] = end_b.y;
	coeff_refinedr[5] = end_b.z;

	previous_coeffr = coeff_refinedr;
	return coeff_refinedr;
}

void RoadLineProcessing::line_detection(cv::Mat img)
{
	if (process_stage == false)
		imgConfig(img.cols, img.rows);

	std::cout << " line Detection Inicialized... " << std::endl;

	static int count = -1;
	count++;
	std::map<int, double> used_lines;
	int dim = 7;

	cv::Point2f start, end;
	cv::Vec3b intensity;

	unsigned int x, y, i;
	unsigned int X = img.cols; /* x image size */
	unsigned int Y = img.rows; /* y image size */

	(*offset)(0) = (*mb_ant)(1) + 31 * sqrt(pow((*mb_ant)(0), 2) + 1);
	(*offset)(1) = (*mb_ant)(1) - 31 * sqrt(pow((*mb_ant)(0), 2) + 1);

	speed_control(ctr_sp);

	for (y = 0; y < Y; y++)
		for (x = 0; x < img.cols; x++)
		{
			intensity = img.at<cv::Vec3b>(y, x);

			if ((x < ((y) * (*mb_ant)(0) + ((*offset)(1)) - 5) || x > ((y) * (*mb_ant)(0) + ((*offset)(0) + 5))) &&
				((t_ant - dt_lAz) < th_az))
				image[x + y * X] = 0;
			else
				image[x + y * X] = (double)(intensity.val[0] + intensity.val[1] + intensity.val[2]) / 3.0;
		}

	/* LSD call */

	int n = 0;

	double *out = LineSegmentDetection(&n, image, X, Y,
									   1.0,  //scale
									   1.0,  //sigma_coef
									   2.0,  // quant
									   22.5, // "ang_th"),
									   2.0,  // "log_eps"),
									   0.7,  //"density_th"),
									   1024, // "n_bins"),
									   NULL,
									   NULL, NULL);

	printf("Size img( %d x %d )\nNumber of line segments detected: %d\n", img.cols, img.rows, n);

	double m, b, distance;
	int index;
	int n_used = 0;

#ifdef DEBUG
	cv::line(img, cv::Point2f(((*offset)(1)), 0), cv::Point2f((128 * (*mb_ant)(0) + (*offset)(1)), 128), cv::Scalar(255, 153, 0), 1, 8);
	cv::line(img, cv::Point2f(((*offset)(0)), 0), cv::Point2f((128 * (*mb_ant)(0) + (*offset)(0)), 128), cv::Scalar(0, 153, 255), 1, 8);
#endif

	for (i = 0; i < n; i++)
	{

		start = cv::Point2d(out[i * dim + 0], out[i * dim + 1]);
		end = cv::Point2d(out[i * dim + 2], out[i * dim + 3]);

		index = round_int(start.y) * X + round_int(start.x);

		//find the lines

		if (((start.x < ((start.y) * (*mb_ant)(0) + (*offset)(1)) || start.x > ((start.y) * (*mb_ant)(0) + (*offset)(0))) ||
			 (end.x < ((end.y) * (*mb_ant)(0) + (*offset)(1)) || end.x > ((end.y) * (*mb_ant)(0) + (*offset)(0)))) &&
			((t_ant - dt_lAz) < th_az))
		{
			continue;
		}

		//find the line equation
		m = (double)(end.x - start.x) / (end.y - start.y);

		if (!(std::abs(m) > 0.05 && std::abs(m) < 10))
			continue;

		n_used++;
		used_lines[i + 1] = m;

#ifdef DEBUG
		cv::line(img, start, end, cv::Scalar(255, 0, 255), 1, 8);
#endif
	}

	printf(" Detected Image Lines =  %d  \n", (int)used_lines.size());

	int dim_a = 7;
	double mod, mod_p = 0;
	// pcl::PointXYZ start_aux, end_aux, start_b, end_b;
	PointXYZ start_aux, end_aux, start_b, end_b;
	VectorXf liner(6);

	std::map<int, double>::iterator it = used_lines.begin();

	if (it != used_lines.end())
	{
		for (; it != used_lines.end(); ++it)
		{
			int index = it->first - 1;

			start_aux.x = (float)out[index * dim_a + 0];
			start_aux.y = (float)out[index * dim_a + 1];
			start_aux.z = 0.0;
			end_aux.x = (float)out[index * dim_a + 2];
			end_aux.y = (float)out[index * dim_a + 3];
			end_aux.z = 0.0;

			mod = sqrt((start_aux.x - end_aux.x) * (start_aux.x - end_aux.x) +
					   (start_aux.y - end_aux.y) * (start_aux.y - end_aux.y));

			if (mod > mod_p)
			{
				mod_p = mod;
				start_b = start_aux;
				end_b = end_aux;
			}
		}
	}
	else
		liner = previous_coeffr;

	liner[0] = start_b.x;
	liner[1] = start_b.y;
	liner[2] = start_b.z;

	liner[3] = end_b.x;
	liner[4] = end_b.y;
	liner[5] = end_b.z;

	previous_coeffr = liner;

	// Linhas detectadas na imagem corrente (COR LARANJA)
	cv::Point2f startR, endR;

	if (liner.size())
	{
		startR.x = liner[0];
		startR.y = liner[1];
		endR.x = liner[3];
		endR.y = liner[4];

		invert_point(&startR, &endR);
	}
	else if (count == 0)
		count = -1;

#ifdef DEBUG //	Linha LARANJA
	cv::line(img, startR, endR, cv::Scalar(0, 100, 255), 3, 8);
#endif

	// Linhas obtidas depois de aplicar uma filtragem temporal (COR AZUL)

	if (count && ((t_ant - dt_lAz) < th_az))
		temporal_line_filtering(&startR, &endR, &previous_Rline, 0.3);
	else
		init_temporal_line_filtering(&startR, &endR, &previous_Rline);

#ifdef DEBUG // Linha AZUL
	cv::line(img, startR, endR, cv::Scalar(255, 0, 0), 2, 8);
#endif

	printf(" Start\n\t(x): %2.2f\t(y):%2.2f\n End\n\t(x): %2.2f\t(y): %2.2f\n\n", startR.x, startR.y, endR.x, endR.y);

	std::cout << " line Detection Finished... " << std::endl;

	//find the line equation
	m = (double)(endR.x - startR.x) / (endR.y - startR.y);
	b = (double)startR.x - m * startR.y;

	if ((abs(m - (*mb_ant)(0)) < 2.5) || ((t_ant - dt_lAm) > th_am))
	{
		dt_lAm = ros::Time::now().toSec();
		double sigma_m = 0.65, sigma_b = 0.65;
		(*mb_ant)(0) = sigma_m * (*mb_ant)(0) + (1 - sigma_m) * m;
		(*mb_ant)(1) = sigma_b * (*mb_ant)(1) + (1 - sigma_b) * b;
	}
	else
	{
		(*mb_ant)(0) = m;
		(*mb_ant)(1) = b;
	}

	if ((*mb_ant)(0) != (*mb_ant)(0))
	{
		(*mb_ant)(0) = m;
		(*mb_ant)(1) = b;
	}

	std::cout << (*mb_ant)(0) << ' ' << (*mb_ant)(1) << '\n';

	cv::Point2f statePt2(((*mb_ant)(0) * (*desired_line)(1) + (*mb_ant)(1)), (*desired_line)(1));
	cv::Point2f statePt1(((*mb_ant)(0) * 0.0 + (*mb_ant)(1)), 0.0);

	this->steering = visual_servoing_control(statePt1, statePt2);

#ifdef DEBUG

	int pos = 0;
	cv::Scalar scalar = cv::Scalar(0, 255, 0);

	std::stringstream ss;

	double m1 = (statePt2.x - statePt1.x) / (statePt2.y - statePt1.y);
	double b1 = statePt1.x - m1 * statePt1.y;

	cv::Point2f observed_pt(m1 * (*desired_line)(1) + b1, (*desired_line)(1));

	ss.str(std::string());
	ss << "Delta[rad]: " << this->steering;
	cv::putText(img, ss.str().c_str(), cv::Point2d(10, pos += 10), cv::FONT_HERSHEY_SIMPLEX, 0.35, scalar, 1);

	ss.str(std::string());
	ss << "Delta[deg]: " << this->steering * 180 / M_PI;
	cv::putText(img, ss.str().c_str(), cv::Point2d(10, pos += 10), cv::FONT_HERSHEY_SIMPLEX, 0.35, scalar, 1);

	ss.str(std::string());
	ss << "Error_control: " << (*error_control)(0);
	cv::putText(img, ss.str().c_str(), cv::Point2d(10, pos += 10), cv::FONT_HERSHEY_SIMPLEX, 0.35, scalar, 1);

	ss.str(std::string());
	ss << "m: " << m1;
	cv::putText(img, ss.str().c_str(), cv::Point2d(10, pos += 10), cv::FONT_HERSHEY_SIMPLEX, 0.35, scalar, 1);

	ss.str(std::string());
	ss << "Am: " << (t_ant - dt_lAm);
	cv::putText(img, ss.str().c_str(), cv::Point2d(10, pos += 10), cv::FONT_HERSHEY_SIMPLEX, 0.35, scalar, 1);

	ss.str(std::string());
	ss << "Az: " << (t_ant - dt_lAz);
	cv::putText(img, ss.str().c_str(), cv::Point2d(10, pos += 10), cv::FONT_HERSHEY_SIMPLEX, 0.35, scalar, 1);

	ss.str(std::string());
	ss << format("v: %0.3f dt_v: %0.3f", velocity, dt_v);
	cv::putText(img, ss.str().c_str(), cv::Point2d(10, pos += 10), cv::FONT_HERSHEY_SIMPLEX, 0.35, scalar, 1);

	/*
	ss.str(std::string());
	ss << "Desired_line: " << (*desired_line)(0) << ", " << (*desired_line)(1);
	cv::putText(img, ss.str().c_str(), cv::Point2d(10, pos+=10), cv::FONT_HERSHEY_SIMPLEX, 0.35, scalar, 1);

	ss.str(std::string());
	ss << "pt1: " << statePt1 << " pt2: " << statePt2;
	cv::putText(img, ss.str().c_str(), cv::Point2d(10, pos+=10), cv::FONT_HERSHEY_SIMPLEX, 0.35, scalar, 1);

	ss.str(std::string());
	ss << "Observed_pt: " << observed_pt;
	cv::putText(img, ss.str().c_str(), cv::Point2d(10, pos+=10), cv::FONT_HERSHEY_SIMPLEX, 0.35, scalar, 1);

*/
	cv::line(img, statePt1, statePt2, cv::Scalar(0, 255, 191), 2, 8);
	cv::line(img, cv::Point2f((*desired_line)(0), (*desired_line)(1)), cv::Point2f((*desired_line)(0), 0), cv::Scalar(255, 0, 255), 1, 8);

	cv::imshow("view", img);
	cv::waitKey(3);
#endif

	free((void *)out);
}

double RoadLineProcessing::visual_servoing_control(cv::Point2f statePt1, cv::Point2f statePt2)
{

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// VISUAL SERVOING CONTROL
	////////////////////////////////////////////////////////////////////////////////////////////////////

	std::cout << " pt1: " << statePt1 << " pt2: " << statePt2 << " Velocity: " << velocity << std::endl;

	//find the line equation
	double m = (statePt2.x - statePt1.x) / (statePt2.y - statePt1.y);
	double b = statePt1.x - m * statePt1.y;

	cv::Point2f observed_pt(m * (*desired_line)(1) + b, (*desired_line)(1));

	float observed_angle = atan(m);

	double u, dt = 0;
	u = ros::Time::now().toSec();
	dt = u - t_ant;
	t_ant = u;

	(*error_control)(0) = (*desired_line)(0) - observed_pt.x; // estimated X error

	ex_sum += (*error_control)(0) * dt; //integral

	ex_sum = std::max(-40.0, std::min(ex_sum, 40.0)); //stauration integral

	u = (*error_control)(0) * kp + kd * ((*error_control)(0) - ex_ant) / dt + ki * ex_sum;
	ex_ant = (*error_control)(0);
	(*error_control)(0) = u;
	(*error_control)(1) = (atan((*desired_line)(2)) - observed_angle) * kp;

	double lambda = 1;
	double pho = 30.0 * M_PI / 180.0; // radians
	double ty = 0.13f;				  //1.4f;	// meters
	double tz = 0.23f;				  //2.2f;	// meters
	double v = velocity;			  // m/s
	double c_x = 160.00;			  //320.00;	//389.6477;
	double c_y = 64.00;				  //240.00;	//137.8856;

	double delta = controller(0.0,				   //(*desired_line)(4), // Theta reference
							  0.0,				   // X point reference
							  0.0,				   // Y point reference
							  (*error_control)(1), // theta error
							  (*error_control)(0), //  X error
							  lambda,			   // lambda paramenter of the controller
							  pho,				   // pho: tilt angle of the camera
							  ty,				   // y axis translation to camera reference
							  tz,				   // z axis translation to camera reference
							  v					   // speed...
	);
	delta = -delta;

	// double driver_wheel = delta * 2.26709332 ; // // Steering driver wheel
	// driver_wheel = std::max(std::min(1.0,driver_wheel),-1.0) * -1.0f;

	//cv::Point2f statePt2 = statePt1 + vector * 200;
	std::cout << " Time DeltaT (dt) : " << dt << "\n Desired_point: " << cv::Point2f((*desired_line)(0), (*desired_line)(1)) << " Desired Angle: " << atan((*desired_line)(2)) * 180 / M_PI << "\n Observerd_pt: " << observed_pt << " Observed Angle: " << observed_angle * 180 / M_PI << "\n ERROR (X): " << (*desired_line)(0) - observed_pt.x << " (theta) " << (atan((*desired_line)(2)) - observed_angle) << "\n Delta: " << delta * 180 / M_PI << " [degree] \n delta: " << delta << " [rad] " << std::endl;

	delta = std::max(std::min(delta, (double)0.6), (double)-0.6); // 0.6[rad] 34,3775[raus]
	// delta = std::max(std::min(delta, (double)0.436333), (double)-0.436333); // 0.436333[rad] 25[graus]

	//return driver_wheel;
	return delta; // [rad]
}

void RoadLineProcessing::init_temporal_line_filtering(cv::Point2f *start, cv::Point2f *end, VectorXf *line)
{
	//fix the desired line equation
	//find line equation

	dt_lAz = ros::Time::now().toSec();
	float m = (end->y - start->y) / (end->x - start->x);
	float b = start->y - m * start->x;

	(*line)[2] = m;
	(*line)[3] = b;

	// x = (y - b )/ m;

	// y = mx + b

	(*line)[1] = heigh_image;
	(*line)[0] = ((*line)[1] - (*line)[3]) / (*line)[2]; //x = (y - b) / m

	end->x = (*line)[0];
	end->y = (*line)[1];

	start->x = (0 - (*line)[3]) / (*line)[2]; //x = (y - b) / m
	start->y = 0;							  // y = mx + b
}

void RoadLineProcessing::temporal_line_filtering(cv::Point2f *start, cv::Point2f *end, VectorXf *line, double weight)
{

	//fix the desired line equation
	//find line equation
	float m = (end->y - start->y) / (end->x - start->x);
	float b = start->y - m * start->x;

	ctr_sp = false;
	// printf(" m: %2.2f  ant_m: %2.2f  diff: %2.2f diff_2: %2.2f \n",m,previous_Lline(2), m-(*line)(2) , sqrt((m-(*line)(2)) * (m-(*line)(2))));
	if (sqrt(pow((m - (*line)[2]), 2)) < weight)
	{
		dt_lAz = ros::Time::now().toSec();
		ctr_sp = true;
		// previous_m = sigma * m_previous_m + (1-sigma) * m
		(*line)[2] = sigma_m * (*line)[2] + (1 - sigma_m) * m;
		(*line)[3] = sigma_b * (*line)[3] + (1 - sigma_b) * b;

		// x = (-b + y)/ m;
		(*line)[0] = (-(*line)[3] + (*line)[1]) / (*line)[2];
	}

	end->x = (*line)[0];
	end->y = (*line)[2] * (*line)[0] + (*line)[3]; // y = mx + b

	start->x = (0 - (*line)[3]) / (*line)[2]; // x = (-b + y) / m
	start->y = 0;							  // y = mx + b
}

void RoadLineProcessing::invert_point(cv::Point2f *start, cv::Point2f *end)
{
	if (end->y < start->y)
	{
		float aux = end->y;
		end->y = start->y;
		start->y = aux;

		aux = end->x;
		end->x = start->x;
		start->x = aux;
	}
}

void RoadLineProcessing::grab(cv::Mat image)
{
	line_detection(image);
}

void RoadLineProcessing::speed_control(bool gs)
{
	double dt = ros::Time::now().toSec();
	-t_ant;

	dt_v += dt;

	if (gs)
	{
		if (velocity >= v_set)
		{
			velocity = v_set;
			dt_v = 0.0;
		}
		else if (dt_v > 0.1)
		{
			velocity+=0.1;
			dt_v = 0.0;
		}
	}
	else
	{
		if (velocity <= 0)
		{
			velocity = 0;
			dt_v = 0.0;
		}
		else if (dt_v > 0.1)
		{
			velocity-=0.2;
			dt_v = 0.0;
		}
	}
}

/**
@brief
ROS callback that return the image from the topic
@param msg sensor_msgs::Image

**/
void RoadLineProcessing::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{

	try
	{
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		line_detection(cv_ptr->image);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}