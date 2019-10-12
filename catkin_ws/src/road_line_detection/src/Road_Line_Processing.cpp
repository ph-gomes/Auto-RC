#include <stdio.h>

#include "Road_Line_Processing.hpp"
#include <fstream> // std::fstream

bool inOffset(std::vector<double> coeffs, cv::Point2f p);
void invert_point(cv::Point2f *start, cv::Point2f *end);
std::string format(const char *const format, ...);

void LSDimage(double *image, cv::Mat &img);
cv::Mat ColorFilter(cv::Mat input);
void calcHistogram(int &idMax, std::vector<int> &histogram, cv::Mat img);
void slidingWindowFit(cv::Mat img, cv::Mat &filtered_img, cv::Mat &histogramImg,
											int idMax,
											std::vector<double> &x_points,
											std::vector<double> &y_points);
void lineFit(cv::Mat img, cv::Mat &lineFit_img, cv::Mat &filtered_img,
						 std::vector<double> coeff,
						 std::vector<double> &x_points, std::vector<double> &y_points);
std::vector<double> polyFitPoints(std::vector<double> &x_points,
																	std::vector<double> &y_points, int p_y = 120);

RoadLineProcessing::RoadLineProcessing(ros::NodeHandle nh)
{

	desired_line = new cv::Mat_<float>(5, 1);
	error_control = new cv::Mat_<float>(2, 1);

	offset = new cv::Mat_<float>(2, 1);

	p_y = 90;

	// Vector Information: [0]:a, [1]:b, [2]:c || a + bx + cx²
	previous_poly.resize(4);
	previous_poly[0] = 160.0;
	previous_poly[1] = 0.0;
	previous_poly[2] = 0.0;
	previous_poly[3] = previous_poly[1] + 2 * previous_poly[2] * p_y;

	process_stage = true;
	initializer = false;

	// Point Information (x, y) // Image Size : 320 x 128
	cv::Point2f start(160, 0), end(160, 128);

	float m = (double)(end.x - start.x) / (end.y - start.y); // 0
	float b = (double)start.x - m * start.y;								 // 160

	float desired_y = 128.0;
	float desired_x = m * desired_y + b; // 160

	// Vector Information: [0]:x, [1]:y, [2]:m, [3]:b
	(*desired_line)(0) = desired_x;
	(*desired_line)(1) = desired_y;
	(*desired_line)(2) = m;
	(*desired_line)(3) = b;

	ex_ant = 0;
	ex_sum = 0;

	dt_ctr = ros::Time::now().toSec();
	t_ant = ros::Time::now().toSec();
	t_res = ros::Time::now().toSec();

	//Poderar a filtragem temporal
	sigma = 0.10;		 // sigma    = 0.65;
	sigma_x = 0.10;	// sigma_x  = 0.65;
	sigma_xx = 0.10; // sigma_xx = 0.65;

	nh.param("k_p", kp, 0.0250);
	nh.param("k_i", ki, 0.00);
	nh.param("k_d", kd, 0.00);

	v_set = 1.70;
	dt_v = 0.0;

	velocity = 1.66;
	ctr_sp = 0;

	GB_Size = 5;
	weight = 0.5;

	std::cout << "ki: " << ki << " kd: " << kd << " kp: " << kp << std::endl;

	// cv::namedWindow("original view", CV_WINDOW_KEEPRATIO);
	// cv::namedWindow("transformed view", CV_WINDOW_KEEPRATIO);
	// cv::namedWindow("img_threshold");
	cv::startWindowThread();
};

RoadLineProcessing::~RoadLineProcessing()
{
	free((void *)image);
	delete (desired_line);

	// cv::destroyWindow("original view");
	// cv::destroyWindow("img_threshold");
}

void RoadLineProcessing::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		cv_bridge::CvImagePtr cv_ptr =
				cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		line_detection(cv_ptr->image);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void RoadLineProcessing::initialConfig(int X, int Y)
{
	// Reserva espaço para a imagem do LSD
	image = (double *)malloc((size_t)(X * Y) * sizeof(double));
	initializer = true;

	// Pontos para o bird eye
	cv::Point2f inputQuad[4];
	cv::Point2f outputQuad[4];

	// inputQuad[0] = cv::Point2f(96, 200);
	// inputQuad[1] = cv::Point2f(152, 130);
	// inputQuad[2] = cv::Point2f(217, 130);
	// inputQuad[3] = cv::Point2f(288, 200);

	// outputQuad[0] = cv::Point2f(96, 200);
	// outputQuad[1] = cv::Point2f(96, 0);
	// outputQuad[2] = cv::Point2f(288, 0);
	// outputQuad[3] = cv::Point2f(288, 200);

	//384 x 200
	//320 x 128

	// inputQuad[0] = cv::Point2f(80, 128);
	// inputQuad[1] = cv::Point2f(127, 83);
	// inputQuad[2] = cv::Point2f(181, 83);
	// inputQuad[3] = cv::Point2f(240, 128);

	inputQuad[0] = cv::Point2f(115, 128);
	inputQuad[1] = cv::Point2f(115, 0);
	inputQuad[2] = cv::Point2f(240, 0);
	inputQuad[3] = cv::Point2f(240, 128);

	outputQuad[0] = cv::Point2f(115, 128);
	outputQuad[1] = cv::Point2f(115, 0);
	outputQuad[2] = cv::Point2f(240, 0);
	outputQuad[3] = cv::Point2f(240, 128);

	// Gera as matrizes de transformações
	this->lambda = cv::getPerspectiveTransform(inputQuad, outputQuad);
	this->r_lambda = cv::getPerspectiveTransform(outputQuad, inputQuad);

	printf("%d x %d\n", X, Y);
}

void RoadLineProcessing::line_detection(cv::Mat img)
{

	// Primeira configuração
	if (initializer == false)
		initialConfig(img.cols, img.rows);

	cv::Mat img_mod;

	cv::warpPerspective(img, img_mod, lambda, img.size());

	LSDimage(image, img_mod);

	/* ==========================================================================
	*		Verificar se faz sentido o uso de ERODE e DILATE
	*  ==========================================================================
	* 
	* cv::namedWindow("before", CV_WINDOW_KEEPRATIO);
	* cv::imshow("before", img_mod);
	* 
	* cv::erode(img_mod,
	* 					img_mod,
	* 					cv::Mat::ones(3, 3, CV_8U),
	* 					cv::Point(-1, -1),
	* 					2);
	* cv::dilate(img_mod,
	* 					 img_mod,
	* 					 cv::Mat::ones(3, 3, CV_8U),
	* 					 cv::Point(-1, -1),
	* 					 3);
	* 
	* cv::namedWindow("after", CV_WINDOW_KEEPRATIO);
	* cv::imshow("after", img_mod);
	*/

	std::vector<double> x_points;
	std::vector<double> y_points;

	if (process_stage)
	{
		std::vector<int> histogram(img_mod.cols);
		int idMax;
		calcHistogram(idMax, histogram, img_mod);

		/* Plota Histograma na Imagem */
		cv::Mat histogramImg;
		img_mod.copyTo(histogramImg);
		for (int i = 0; i < histogramImg.cols; i++)
		{
			double alpha = 0.5;
			cv::line(histogramImg, cv::Point(i, histogramImg.rows),
							 cv::Point(i, histogramImg.rows - histogram[i]),
							 cv::Scalar(0, 0, 255), 1, 8);
			cv::addWeighted(histogramImg, alpha, histogramImg, 1 - alpha, 0, histogramImg);
		}
		/* Marca o ponto máximo do histograma */
		cv::circle(histogramImg, cv::Point(idMax, histogramImg.rows - histogram[idMax]),
							 4, cv::Scalar(255, 0, 0), -1, 8);

		/* Mat contendo somente os pontos interessantes*/
		cv::Mat slidingWindowFit_img(img_mod.size(), CV_8UC3, cv::Scalar(0, 0, 0));

		slidingWindowFit(img_mod, slidingWindowFit_img, histogramImg, idMax, x_points, y_points);

		std::vector<double> coeff;

		if (!x_points.empty())
		{
			coeff = polyFitPoints(x_points, y_points);

			for (double i = 0; i < histogramImg.rows; i += histogramImg.rows / 10.0)
			{
				double j = i + histogramImg.rows / 10.0;
				cv::line(histogramImg,
								 cv::Point(coeff[0] + coeff[1] * i + coeff[2] * i * i, i),
								 cv::Point(coeff[0] + coeff[1] * j + coeff[2] * j * j, j),
								 cv::Scalar(255, 0, 0), 2, 8);
			}

			for (int i = 0; i < coeff.size(); i++)
			{
				if (std::isnan(coeff[i]))
				{
					coeff[i] = previous_poly[i];
					printf("NaN capture");
				}
				else if (std::isinf(coeff[i]))
				{
					coeff[i] = previous_poly[i];
					printf("Inf capture");
				}
			}

			previous_poly = coeff;
		}

		/* Histograma */
		cv::namedWindow("histograma", CV_WINDOW_KEEPRATIO);
		cv::imshow("histograma", histogramImg);
		/* slidingWindowFit */
		cv::namedWindow("slidingWindowFit", CV_WINDOW_KEEPRATIO);
		cv::imshow("slidingWindowFit", slidingWindowFit_img);
	}

	cv::Mat lineFit_img(img_mod.size(), CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat overlay_line, overlay(img_mod.size(), CV_8UC3, cv::Scalar(0, 0, 0));
	img_mod.copyTo(overlay_line);

	lineFit(img_mod, lineFit_img, overlay_line, previous_poly, x_points, y_points);

	if (!x_points.empty())
	{
		std::vector<double> temp = polyFitPoints(x_points, y_points);

		previous_poly[0] = previous_poly[0] * 0.6 + temp[0] * 0.4;
		previous_poly[1] = previous_poly[1] * 0.6 + temp[1] * 0.4;
		previous_poly[2] = previous_poly[2] * 0.6 + temp[2] * 0.4;
		previous_poly[3] = previous_poly[3] * 0.6 + temp[3] * 0.4;

		for (int i = 0; i < previous_poly.size(); i++)
		{
			if (std::isnan((int)previous_poly[i]))
			{
				previous_poly[i] = 0;
				printf("NaN capture");
			}
			if (std::isinf((int)previous_poly[i]))
			{
				previous_poly[i] = 0;
				printf("Inf capture");
			}
		}

		for (double i = 0; i < overlay_line.rows; i += overlay_line.rows / 10.0)
		{
			double j = i + overlay_line.rows / 10.0;
			cv::line(overlay_line,
							 cv::Point(previous_poly[0] + previous_poly[1] * i + previous_poly[2] * i * i, i),
							 cv::Point(previous_poly[0] + previous_poly[1] * j + previous_poly[2] * j * j, j),
							 cv::Scalar(255, 0, 0), 2, 8);
			cv::line(overlay,
							 cv::Point(previous_poly[0] + previous_poly[1] * i + previous_poly[2] * i * i, i),
							 cv::Point(previous_poly[0] + previous_poly[1] * j + previous_poly[2] * j * j, j),
							 cv::Scalar(255, 0, 0), 2, 8);
		}

		this->process_stage = false;
	}
	else
	{
		this->process_stage = true;
	}

	int y = 120;
	cv::Point2f observed_pt(previous_poly[0] + previous_poly[1] * y + previous_poly[2] * y * y, y);
	if (std::isnan(previous_poly[3]))
		previous_poly[3] = 0;
	this->steering = RoadLineProcessing::visual_servoing_control(observed_pt, previous_poly[3]);
	printf("\tdif_pontos: %3.2f dif_angulo[deg]: %3.2f°\n\tcomando[rad]: %3.2frad comando[deg]: %3.2f°\n",
				 ((*error_control)(0)), ((*error_control)(1)) * 180 / M_PI,
				 this->steering, this->steering * 180 / M_PI);

	cv::circle(overlay_line, observed_pt, 4, cv::Scalar(0, 255, 0), -1, 8);

	/* Desfaz bird eye */
	cv::warpPerspective(overlay, overlay, r_lambda, overlay.size());
	/* Insere na imagem original */
	cv::addWeighted(overlay, 1, img, 1, 0, overlay);

	/* Imagem original com a linha */
	cv::namedWindow("view", CV_WINDOW_KEEPRATIO);
	cv::imshow("view", overlay);

	cv::namedWindow("lineFit_img", CV_WINDOW_KEEPRATIO);
	cv::imshow("lineFit_img", lineFit_img);
	cv::namedWindow("overlay_line", CV_WINDOW_KEEPRATIO);
	cv::imshow("overlay_line", overlay_line);
	/* Desfaz bird eye */
	cv::warpPerspective(overlay_line, overlay_line, r_lambda, overlay_line.size());
	/* Insere na imagem original */
	cv::addWeighted(overlay_line, 1, img, 1, 0, overlay_line);
	cv::line(overlay_line,
					 cv::Point((*desired_line)(0), 0),
					 cv::Point((*desired_line)(0), (*desired_line)(1)),
					 cv::Scalar(255, 255, 0), 2, 8);

	/* lineFit */
	cv::namedWindow("lineFit", CV_WINDOW_KEEPRATIO);
	cv::imshow("lineFit", overlay_line);
	cv::waitKey(3);
}

double RoadLineProcessing::visual_servoing_control(cv::Point2f observed_pt, float observed_angle)
{

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// VISUAL SERVOING CONTROL
	////////////////////////////////////////////////////////////////////////////////////////////////////

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
	// (*error_control)(1) = (atan((*desired_line)(2)) - observed_angle) * kp;

	double lambda = 1;
	double pho = 30.0 * M_PI / 180.0; // radians
	double ty = 0.06f;								//0.13f;	//1.4f;	// meters
	double tz = 0.06f;								//0.23f;	//2.2f;	// meters
	double v = 1.0f;									// m/s
	double c_x = 160.00;							//320.00;	//389.6477;
	double c_y = 120.00;							//240.00;	//137.8856;

	double delta = controller(0.0,								 //(*desired_line)(4), // Theta reference
														0.0,								 // X point reference
														0.0,								 // Y point reference
														(*error_control)(1), // theta error
														(*error_control)(0), //  X error
														lambda,							 // lambda paramenter of the controller
														pho,								 // pho: tilt angle of the camera
														ty,									 // y axis translation to camera reference
														tz,									 // z axis translation to camera reference
														v										 // speed...
	);
	// delta = -delta;

	delta = std::max((double)-0.5, std::min(delta, (double)0.5)); // 0.6[rad] 34,3775[graus]
	std::cout << " Time DeltaT (dt) : " << dt << "\n Desired_point: " << cv::Point2f((*desired_line)(0), (*desired_line)(1)) << " Desired Angle: " << atan((*desired_line)(2)) * 180 / M_PI << "\n Observerd_pt: " << observed_pt << " Observed Angle: " << observed_angle * 180 / M_PI << "\n ERROR (X): " << (*desired_line)(0) - observed_pt.x << " (theta) " << (atan((*desired_line)(2)) - observed_angle) << "\n Delta: " << delta * 180 / M_PI << " [degree] \n delta: " << delta << " [rad] " << std::endl;

	//return driver_wheel;
	return delta; // [rad]
}

/* Formata String */
std::string format(const char *const format, ...)
{
	auto temp = std::vector<char>{};
	auto length = std::size_t{63};
	std::va_list args;
	while (temp.size() <= length)
	{
		temp.resize(length + 1);
		va_start(args, format);
		const auto status = std::vsnprintf(temp.data(), temp.size(), format, args);
		va_end(args);
		if (status < 0)
			throw std::runtime_error{"string formatting error"};
		length = static_cast<std::size_t>(status);
	}
	return std::string{temp.data(), length};
}

cv::Mat ColorFilter(cv::Mat input)
{
	cv::Mat HSV_filter;

	cv::cvtColor(input, HSV_filter, cv::COLOR_BGR2HSV);

	cv::namedWindow("HSV_filter", CV_WINDOW_KEEPRATIO);
	cv::imshow("HSV_filter", HSV_filter);
	cv::waitKey(100);

	cv::inRange(HSV_filter, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 90), HSV_filter);
	// cv::inRange(HSV_filter, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 50), HSV_filter);

	cv::cvtColor(HSV_filter, HSV_filter, cv::COLOR_GRAY2BGR);

	return HSV_filter;
}

bool inOffset(std::vector<double> coeffs, cv::Point2f p)
{
	double l = (coeffs)[0] + (coeffs)[1] * p.y + (coeffs)[2] * p.y * p.y - 100,
				 r = (coeffs)[0] + (coeffs)[1] * p.y + (coeffs)[2] * p.y * p.y + 100;

	return (p.x > l && p.x < r) ? true : false;
}

void LSDimage(double *image, cv::Mat &img)
{
	for (unsigned int y = 0; y < img.rows; y++)
		for (unsigned int x = 0; x < img.cols; x++)
		{
			cv::Vec3b intensity = img.at<cv::Vec3b>(y, x);
			image[x + y * img.cols] =
					(double)(intensity.val[2] + intensity.val[1] + intensity.val[0]) / 3.0;
		}
	int n;
	double *out = LineSegmentDetection(&n, image, img.cols, img.rows,
																		 0.5,	//scale
																		 1.0,	//sigma_coef
																		 2.0,	// quant
																		 22.5, // "ang_th"),
																		 2.0,	// "log_eps"),
																		 0.7,	//"density_th"),
																		 1024, // "n_bins"),
																		 NULL,
																		 NULL, NULL);

	img = ColorFilter(img);

	int dim = 7;
	for (unsigned int i = 0; i < n; i++)
	{
		cv::Point start = cv::Point(out[i * dim + 0], out[i * dim + 1]);
		cv::Point end = cv::Point(out[i * dim + 2], out[i * dim + 3]);

		// cv::line(img, start, end, cv::Scalar(0, 0, 255), 2, 8);

		double m = (double)(end.x - start.x) / (end.y - start.y);
		double b = (double)start.x - m * start.y;

		if (!(std::abs(m) > 0.05 && std::abs(m) < 10))
			continue;

		double distance = std::sqrt((end.x - start.x) * (end.x - start.x) + (end.y - start.y) * (end.y - start.y));

		if (distance < 20)
			continue;

		// cv::line(img, start, end, cv::Scalar(255, 255, 255), 2, 8);
	}
}

void calcHistogram(int &idMax, std::vector<int> &histogram, cv::Mat img)
{
	idMax = 0;
	for (int x = 0; x < img.cols; x++)
	{
		for (int y = 0; y < img.rows; y++)
		{
			cv::Vec3b intensity = img.at<cv::Vec3b>(y, x);
			if (!!intensity.val[0])
				histogram[x]++;
		}
		if (histogram[x] > histogram[idMax])
			idMax = x;
	}
}

void slidingWindowFit(cv::Mat img, cv::Mat &filtered_img, cv::Mat &histogramImg,
											int idMax, std::vector<double> &x_points, std::vector<double> &y_points)
{
	int nwindows = 15;
	int window_w = 70;
	int windows_h = img.rows / nwindows;

	int minPix = 25;

	int x_base = idMax;

	for (int i = 0; i < nwindows; i++)
	{
		int win_y_l = img.rows - (i + 1) * windows_h; // 160
		int win_y_h = img.rows - i * windows_h;				// 200

		int win_x_l = x_base - window_w; // 0
		int win_x_h = x_base + window_w; // 50

		cv::rectangle(histogramImg, cv::Point(win_x_l, win_y_l), cv::Point(win_x_h, win_y_h),
									cv::Scalar(0, 255, 0), 2);

		int count = 0;
		int posSum = 0;

		for (int y = win_y_l; y < win_y_h; y++)
			for (int x = win_x_l; x < win_x_h; x++)
			{
				cv::Vec3b intensity = img.at<cv::Vec3b>(y, x);
				if (!!intensity.val[0] && (x >= 0 && x <= img.cols))
				{
					x_points.push_back(x);
					y_points.push_back(y);
					filtered_img.at<cv::Vec3b>(y, x) = intensity;
					count++;
					posSum += x;
				}
			}

		if (count > minPix)
		{
			x_base = posSum / count;
		}
	}
}

std::vector<double> polyFitPoints(std::vector<double> &x_points,
																	std::vector<double> &y_points, int p_y)
{
	if (x_points.empty() || y_points.empty())
		return std::vector<double>();

	std::vector<double> coeff(4);
	polyFit<double>().fitIt(y_points, x_points, 1, coeff);

	/* Ponto de interesse */
	// coeff[3] = coeff[1] + 2 * coeff[2] * p_y;
	// a + bx + cx²
	// b + cx
	// a + bx
	// a
	coeff[3] = atan(coeff[1]);

	return coeff;
}

void lineFit(cv::Mat img, cv::Mat &filtered_img, cv::Mat &overlay_line,
						 std::vector<double> coeff,
						 std::vector<double> &x_points, std::vector<double> &y_points)
{
	int margin = 70;
	x_points.clear();
	y_points.clear();

	for (int y = 0; y < img.rows; y++)
	{
		int X = coeff[0] + coeff[1] * y + coeff[2] * y * y;
		for (int x = ((X - margin) > 0 ? X - margin : 0);
				 x < ((X + margin) < img.cols ? X + margin : img.cols);
				 x++)
		{
			cv::Vec3b intensity = img.at<cv::Vec3b>(y, x);
			overlay_line.at<cv::Vec3b>(y, x) =
					cv::Vec3b(intensity.val[0], 255, intensity.val[2]);

			if (!!intensity.val[0])
			{
				x_points.push_back(x);
				y_points.push_back(y);
				filtered_img.at<cv::Vec3b>(y, x) = intensity;
			}
		}
	}
}