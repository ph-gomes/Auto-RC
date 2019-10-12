/**
 * @file LineProcessing.cpp
 * @brief Programa de imageamento para o controle de um seguidor de linha
 * @author Igor Bonato Matricardi - 34148
 * @author Patrick Cesar Pereira Rocha - 33401
 * @date 10-12-2018
 */

/*! \mainpage Implementação do Seguidor de Linha
*	@brief Programa de imageamento para o controle de um seguidor de linha
* 	@author Igor Bonato Matricardi - 34148
* 	@author Patrick Cesar Pereira Rocha - 33401
* 	@date 10-12-2018
*/

/*
 * LineProcessing.cpp
 *
 *     Author: Giovani Bernardes Vitor, Igor Bonato Matricardi - 34148, Patrick Cesar Pereira Rocha - 33401
 * 	   Compile: catkin_make na pasta 'source'
 * 	   Run: rosrun ecoi24_pdi_2018 ecoi24_pdi_2018_node _image_transport:=compressed
 * 
 * 		Robot: wifi --> password: holaerle
 * 			   ssh erle@10.0.0.1
 * 			   sudo nano /etc/hosts --> 10.0.0.2   IBM-PC
 * 			   sudo gedit /etc/hosts --> 10.0.0.2   IBM-PC
 * 			   
 * 			   Terminal:
 * 			   export ROS_MASTER_URI=http://erle-brain-2:11311/
 * 			   rosservice call /camera/start_capture
 * 
 * 			   SSH:
 * 			   rostopic echo /mavros/state
 * 			   rosrun mavros mavparam set SYSID_MYGCS 1
 * 			   rosrun mavros mavsys mode -c MANUAL
 * 			   rostopic echo /mavros/rc/in
 * 			   
 * 			   Volta Terminal:
 * 			   rosrun ros_erle_cpp_teleoperation_erle_rover teleoperation
 * 			   
 */

#include <stdio.h>
#include "LineProcessing.hpp"

#include <cv.h>			 // open cv general include file
#include <highgui.h> // open cv GUI include file
#include <iostream>	// standard C++ I/O
#include <cmath>
#include <string>

#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv; // OpenCV API is in the C++ "cv" namespace
using namespace std;

double cx, cy, cx_m, cy_m;
double cont = 0;

/**
* @brief constructor of class with initial default parameters
*/
LineProcessing::LineProcessing(ros::NodeHandle nh)
{

	n = &nh;
	erlerover_manager.init();

	rc_override_pub = n->advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

	boost::thread t(boost::bind(&LineProcessing::SendCommandEB2, this));

	cv::namedWindow("view");
	cv::startWindowThread();

	m_ant = b_ant = 0.0;
	width_image = 384; //640;
	heigh_image = 200; //480;

	image = (double *)malloc(width_image * heigh_image * sizeof(double));
	search_mask = (unsigned char *)malloc(width_image * heigh_image * sizeof(unsigned char));

	mask = Mat::zeros(heigh_image, width_image, CV_8UC1);

	cv::line(mask, cv::Point2f(width_image / 2.0, 100), cv::Point2f(width_image / 2.0, 200), cv::Scalar(255, 255, 255), 70, 8);
};

/**
* @brief destructor of class with initial default parameters
*/
LineProcessing::~LineProcessing()
{
	cv::destroyWindow("view");

	free((void *)search_mask);
	free((void *)image);
}

/*
double LineProcessing::linearRegression(std::vector<cv::Point2f> points, double &b, double &m)
{

	double x[] = {1, 2, 4, 3, 5};
	double y[] = {1, 3, 3, 2, 5};
	 
	double b0 = 0;
	double b1 = 0;
	double alpha = 0.01;
	double err=0;

	

	//for (int i = 0; i < 20; i ++) {
	//    int idx = i % 5;
	//    double p = b0 + b1 * x[idx];
	//    err = p - y[idx];
	//    b0 = b0 - alpha * err;
	//    b1 = b1 - alpha * err * x[idx];
	//}

	int loop = 4 * points.size();

	for (int i = 0; i < loop; i ++) {
	    int idx = i % points.size();
	    double p = b0 + b1 * points[idx].x;
	    err = p - points[idx].y;
	    b0 = b0 - alpha * err;
	    b1 = b1 - alpha * err * points[idx].x;

	    //std::cout << " err: " << err << std::endl;
	}

	b = b0;
	m = b1;

	//std::cout << " dentro b: " << b << " m: " << m << std::endl;

	return err;
}
*/

double LineProcessing::linearRegression(std::vector<cv::Point2f> points, double &b, double &m)
{

	int i, n = points.size();

	double xsum = 0, x2sum = 0, ysum = 0, xysum = 0; //variables for sums/sigma of xi,yi,xi^2,xiyi etc
	for (i = 0; i < n; i++)
	{
		xsum = xsum + points[i].x;									 //calculate sigma(xi)
		ysum = ysum + points[i].y;									 //calculate sigma(yi)
		x2sum = x2sum + (points[i].x * points[i].x); //calculate sigma(x^2i)
		xysum = xysum + points[i].x * points[i].y;	 //calculate sigma(xi*yi)
	}
	m = (n * xysum - xsum * ysum) / (n * x2sum - xsum * xsum);		 //calculate slope
	b = (x2sum * ysum - xsum * xysum) / (x2sum * n - xsum * xsum); //calculate intercept

	//double y_fit[n];                        //an array to store the new fitted values of y
	//for (i=0;i<n;i++)
	//    y_fit[i]=a*x[i]+b;                    //to calculate y(fitted) at given x points
	//cout<<"S.no"<<setw(5)<<"x"<<setw(19)<<"y(observed)"<<setw(19)<<"y(fitted)"<<endl;
	//cout<<"-----------------------------------------------------------------\n";
	//for (i=0;i<n;i++)
	//    cout<<i+1<<"."<<setw(8)<<x[i]<<setw(15)<<y[i]<<setw(18)<<y_fit[i]<<endl;//print a table of x,y(obs.) and y(fit.)
	//cout<<"\nThe linear fit line is of the form:\n\n"<<a<<"x + "<<b<<endl;        //print the best fit line
	return 0;
}

void LineProcessing::invert_point(cv::Point2f *start, cv::Point2f *end)
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

/**
 * @brief Extração da linha da imagem capturada
 * 
 * @param msg - retorno dos resultados
 */
void LineProcessing::lineExtraction(const sensor_msgs::ImageConstPtr &msg)
{

	//width_image=msg->width;
	//heigh_image=msg->height;

	//@bug it is not neceesary to convert to opencv image
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// Insert your code here to process LINE DETECTION on the image ////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////

	Mat img, imgGray, imgBlur, imgTh, crop_img;
	int key, dist, A, B, C, D;
	/*if ((cx-30)<0) A=0; else A = cx-30;
	if ((cy-30)<0) B=0; else B = cy-30;
	if ((A+30)>380) C= 380-A; else C = 60;
	if ((B+30)>380) D= 199-B; else B = 60;*/
	img = cv_ptr->image;

	//Mat mask = Mat(msg->width, msg->height, CV_8UC1);
	//if(img.empty()) break;
	//crop_img = img(Rect(20,msg->height-30,msg->width-40,30));

	//imshow("Input", crop_img);

	int Hor_line_thresh = 140;
	int dim = 7;
	double m, b;
	unsigned int x, y, i;					//,j;
	unsigned int X = msg->width;	/* x image size */
	unsigned int Y = msg->height; // - 180; //150 /* y image size */
	int regX = 0, regY = 0;
	int *region;
	cv::Vec3b intensity;
	cv::Point2f start, end;

	for (x = 0; x < msg->width; x++)
		for (y = 0; y < Y; y++)
		{
			intensity = cv_ptr->image.at<cv::Vec3b>(y, x);

			if (y < Hor_line_thresh)
			{
				//search_mask[ x + y * X ] = 	image[ x + y * X ] = 0;
				image[x + y * X] = 0.0;
				//mask.at<char>(y,x) = 0;
			}
			else
			{
				//search_mask[ x + y * X ] = 1;
				//mask.at<char>(y,x) = 255;
				image[x + y * X] = (double)(intensity.val[2] + intensity.val[1] + intensity.val[0]) / 3.0;
			}
		}

	/* LSD call */

	int n = 0;
	int k = 0;
	//double t2=ros::Time::now().toSec();
	//std::cout<<"time1 "<<t2-t1<<"\t";
	//t1=t2;
	///detect the lines
	//double * out = lsd(&n,image,X,Y);

	/* execute LSD */

	double *out = LineSegmentDetection(&n, image, X, Y,
																		 1.0,	//scale
																		 1.0,	//sigma_coef
																		 2.0,	// quant
																		 22.5, // "ang_th"),
																		 2.0,	// "log_eps"),
																		 0.7,	//"density_th"),
																		 1024, // "n_bins"),
																		 &region,
																		 &regX, &regY);

	std::vector<cv::Point2f> pts;
	double ex = 0, ey = 0, sx = 0, sy = 0, count = 0;
	for (i = 0; i < n; i++)
	{
		//out[ i * dim + 0 ]+=roiX;
		//out[ i * dim + 2 ]+=roiX;
		start = cv::Point2d(out[i * dim + 0], out[i * dim + 1]);
		end = cv::Point2d(out[i * dim + 2], out[i * dim + 3]);

		cv::line(img, start, end, cv::Scalar(255, 255, 255), 2, 8);
		//int index= (int)(start.y) * X + (int)(start.x);
		//if(search_mask[index] != 1)
		if (mask.at<char>(start.y, start.x) == 0 && mask.at<char>(end.y, end.x) == 0)
			continue;

		//printf("start.(%f,%f), end(%f,%f) ::: outX1(%f,%f), outX2(%f,%f) \n\n",start.x,start.y,end.x,end.y,out[ i * dim + 0 ],out[ i * dim + 1 ],out[ i * dim + 2 ],out[ i * dim + 3 ]);
		//find the line equation
		m = (double)(end.x - start.x) / (end.y - start.y);
		b = (double)start.x - m * start.y;
		//printf(" m: %f b: %f \n",m,b);
		//m=(out[ i * dim + 2 ]-out[ i * dim + 0 ])/(out[ i * dim + 3 ]-out[ i * dim + 1 ]);
		//b=out[ i * dim + 0 ]-m*out[ i * dim + 1 ];
		//printf(" m: %f b: %f \n\n\n",m,b);

		if (!(std::abs(m) > 0.05 && std::abs(m) < 10))
			continue;

		//cv::line( img,start,end, cv::Scalar( 255,255, 255 ), 2, 8 );

		double distance = std::sqrt((end.x - start.x) * (end.x - start.x) + (end.y - start.y) * (end.y - start.y));

		if (distance < 20)
			continue;

		pts.push_back(start);
		pts.push_back(end);

		break;

		//cv::line( img,start,end, cv::Scalar( 255,255, 255 ), 2, 8 );
	}

	std::cout << "start: " << start.x << " , " << start.y << " end: " << end.x << " , " << end.y << std::endl;

	invert_point(&start, &end);

	//m = (double)(end.x - start.x) / (end.y - start.y);
	//b = (double) start.x - m * start.y;

	//if(abs( start.x - startAnt.x   ) < 80 && abs( end.x - endAnt.x   ) < 80 )
	{
		double sigma = 0.60;
		startAnt = sigma * startAnt + (1 - sigma) * start;
		endAnt = sigma * endAnt + (1 - sigma) * end;
	}

	std::cout << "startAnt: " << startAnt.x << " , " << startAnt.y << " endAnt: " << endAnt.x << " , " << endAnt.y << std::endl;

	cv::line(img, start, end, cv::Scalar(255, 0, 0), 2, 8);
	cv::line(img, startAnt, endAnt, cv::Scalar(0, 0, 255), 2, 8);

	mask = Scalar(0);
	cv::line(mask, startAnt, endAnt, cv::Scalar(255, 255, 255), 60, 8);

	imshow("mask", mask);

	//
	//	cvtColor(crop_img, imgGray, CV_BGR2GRAY);
	//	GaussianBlur(imgGray, imgBlur, Size(7, 7), 0, 0);
	//	threshold(imgBlur, imgTh, 70, 255, THRESH_BINARY_INV);
	//	//adaptiveThreshold(imgBlur, imgTh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 11, 2);
	//	imshow("Threshold", imgTh);
	//
	//	Mat dst, cdst;
	//	Canny(imgTh, dst, 50, 200, 3);
	// 	cvtColor(dst, cdst, CV_GRAY2BGR);

	/*
	vector<Vec4i> lines;
	HoughLinesP(dst, lines, 1, CV_PI/180, 20, 0, 0);
	for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point2d pt1, pt2;
        double m;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;

        pt1.x = cvRound(x0 + 200*(-b)); 
        pt1.y = cvRound(y0 + 200*(a));
        pt2.x = cvRound(x0 - 200*(-b));
        pt2.y = cvRound(y0 - 200*(a));

        line(cdst, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
	}
	*/

	//! Transformada de Linha Probabilística
	//    vector<Vec4i> linesP; //! Armazena os valores da detecção
	//    HoughLinesP(dst, linesP, 1, CV_PI/180, 15, 0, 15 ); //! Detection

	//! Desenha as linhas
	//	int vet[4];
	//    for( size_t i = 0; i < linesP.size(); i++ )
	//    {
	//        Vec4i l = linesP[i];
	//        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
	//		vet[0] = vet[0] + l[0];vet[1] = vet[1] + l[1];vet[2] = vet[2] + l[2];vet[3] = vet[3] + l[3];
	//    }

	//! Evita divisão por zero
	//	if(vet[0]!=0&&vet[1]!=0&&vet[2]!=0&&vet[3]!=0){
	//		vet[0] = vet[0]/linesP.size();
	//		vet[1] = vet[1]/linesP.size();
	//		vet[2] = vet[2]/linesP.size();
	//		vet[3] = vet[3]/linesP.size();}
	///	Point2d ponto1 = Point(vet[0], vet[1]);Point2d ponto2 = Point(vet[2], vet[3]);
	//	line(cdst, ponto1, ponto2, Scalar(255,0,0), 2);

	//! Média da Média no Tempo
	//	int vet_m[4];
	//	vet_m[0] = (vet_m[0]*15+vet[0])/16;
	//	vet_m[1] = (vet_m[1]*15+vet[1])/16;
	//	vet_m[2] = (vet_m[2]*15+vet[2])/16;
	//	vet_m[3] = (vet_m[3]*15+vet[3])/16;
	//	Point2d ponto1_m = Point(vet_m[0], vet_m[1]);Point2d ponto2_m = Point(vet_m[2], vet_m[3]);
	//	line(cdst, ponto1_m, ponto2_m, Scalar(0,255,0), 2);

	/*vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours( imgTh, contours, hierarchy, 1, CHAIN_APPROX_SIMPLE, Point(0,0));

	int largest_area=0;
	int largest_contour_index=0;
	for( int i = 0; i< contours.size(); i++ ){ // iterate through each contour. 
		double a=contourArea( contours[i],false);  //  Find the area of contour
		if(a>largest_area){
			largest_area=a;
			largest_contour_index=i;                //Store the index of largest contour
		}
	}

	Vec4f myline;
	fitLine(contours[largest_contour_index], myline, CV_DIST_L1, 0, 0.01, 0.01);

	Point2d P1, P2;
	P1 = Point(myline[2], myline[3]);
	P2 = Point(myline[2]+myline[0]*40, myline[3]+myline[1]*40);

	Moments mu;

	mu = moments(contours[largest_contour_index], false);
	cx = mu.m10/mu.m00;
	cy = mu.m01/mu.m00;
	cx_m = (cx_m*15+cx)/16;
	cy_m = (cy_m*15+cy)/16;
*/
	/*dist = sqrt(pow((width_image/2)-cx, 2)+pow(60-cy, 2));
	float angle;
	angle = atan2 (cy-(heigh_image/2), cx-(width_image/2))* 180 / M_PI/*asin((float) ((cx-(width_image/2))/dist))*(180.0/M_PI)*/
	// ;
	//angle = -angle;
	/*angle = 90 + angle;
	if(angle > 90)
		angle = -angle;

	stringstream ang, dis;
	ang << angle;

	String text = "Angle: " + ang.str();*/

	//Mat drawing = Mat::zeros( imgTh.size(), CV_8UC3 );
	//drawContours( drawing, contours, largest_contour_index, Scalar( 0, 255, 0 ), 3);

	/*line(drawing, Point(cx, 0), Point(cx, 60), 255);
	line(drawing, Point(0, cy), Point(width_image, cy), 255);
	line(drawing, Point(cx_m, 0), Point(cx_m, 60), Scalar(255,0,255));
	line(drawing, Point(0, cy_m), Point(width_image, cy_m), Scalar(255,0,255));
	//line(drawing, Point((int) cx_m, 0), Point((int)cx_m, 360), Scalar(0,255,255));
	line(drawing, Point(width_image/2, 60), Point(cx, cy), Scalar(0,255,255));
	line(drawing, Point(width_image/2, 0), Point(width_image/2, 60), Scalar(0,0,255), 5);*/

	//line(drawing, P1, P2, Scalar(255,0,255), 8);

	//	double m = (ponto2_m.y - ponto1_m.y) / (ponto2_m.x - ponto1_m.x);
	//	double b = ponto1_m.y - m * ponto1_m.x;

	//	int y60 = (40-b)/m;
	//	int y0  = (-b)/m;
	//	float angle = (atan(m)*180/M_PI);
	//line(drawing, Point(y0, 0), Point(y60, 60), Scalar(255,0,0), 5);

	/*putText(drawing, to_string(angle), Point(2, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(144,0,250), 1, CV_AA);
	putText(drawing, to_string(m), Point(2, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(144,0,250), 1, CV_AA);
	putText(drawing, to_string(b), Point(50, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(144,0,250), 1, CV_AA);*/
	//putText(drawing, to_string(dist), Point(2, 10), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(144,0,250), 1, CV_AA);

	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////

	//cout<<"x1 = "<<(int) ponto1.x <<"\tx2 = "<<(int)ponto2.x<<"\tAngle = "<<angle<<"\terroX: "<<errox<<endl;

	//	int Kp, Vel;
	//	namedWindow( "Contours", WINDOW_AUTOSIZE );
	//	createTrackbar("Kp", "Contours", &Kp, 100);
	//	createTrackbar("Vel", "Contours", &Vel, 100);
	//	int velocidade = Vel + 1500;
	//	int cx = (ponto1_m.x+ponto2_m.x)/2;
	//	int yaw = 1700 + (cx - (width_image-40)/2)*Kp/10;
	//	if(yaw>1900) {yaw = 1900; velocidade = velocidade-1;}
	//	else if(yaw<1100) {yaw = 1100; velocidade = velocidade-1;}
	//	std::cout<<"x1 = "<<(int) ponto1_m.x <<"\tx2 = "<<(int)ponto2_m.x<<"\tcx = "<<cx<<"\tyaw: "<<yaw<<endl;
	//
	//	imshow( "Contours", cdst );
	//
	//	double driver_wheel = visual_servoing_control(ponto1, ponto2 );
	//	static int aux = 0;
	//	erlerover_manager.setAngularVelocity(yaw);
	//	erlerover_manager.setLinearVelocity(1500+Vel);
	//
	cv::imshow("view", cv_ptr->image);
	cv::waitKey(3);

	free((void *)region);
	free((void *)out);
}

/**
 * @brief Visual Servoing Control
 * 
 * @param statePt1 
 * @param statePt2 
 * @return double 
 */
double LineProcessing::visual_servoing_control(cv::Point2f statePt1, cv::Point2f statePt2)
{

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// VISUAL SERVOING CONTROL /////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////

	//cout << "pt1: " << statePt1 << "pt2: " << statePt2 << " Velocity: " << velocity << std::endl;

	//find the line equation
	double m = (statePt2.x - statePt1.x) / (statePt2.y - statePt1.y);
	double b = statePt1.x - m * statePt1.y;

	//cv::Point2f observed_pt(m*(*desired_line)(1) + b,(*desired_line)(1));

	cv::Mat error_control = (cv::Mat_<float>(2, 1) << 0.0, 0.0); // error_x and error_theta

	float observed_angle = atan(m);

	//error_control.at<float>(0) =       (*desired_line)(0) -  observed_pt.x; // estimated X error
	//error_control.at<float>(1) = (atan((*desired_line)(2)) - observed_angle); // estimated theta error

	double lambda = 1;
	double pho = 30.0 / 180.0 * M_PI; // radians
	double ty = 1.4f;									// meters
	double tz = 2.2f;									// meters
	double v = velocity;							// m/s
	double c_x = width_image / 2.0f;
	double c_y = heigh_image / 2.0f;

	//double delta=0;
	double delta = controller(0.0,												//(*desired_line)(4), // Theta reference
														0.0,												// (*desired_line)(0) - c_x,  // X point reference
														0.0,												//(*desired_line)(1) - c_y,  // Y point reference
														error_control.at<float>(1), // theta error
														error_control.at<float>(0), //  X error
														lambda,											// lambda paramenter of the controller
														pho,												// phi: tilt angle of the camera
														ty,													// y axis translation to camera reference
														tz,													// z axis translation to camera reference
														v														// speed
	);

	double driver_wheel;
	//driver_wheel = std::max(std::min(1.0,driver_wheel),-1.0) * -1.0f;

	//std::cout << "Time DeltaT (dt) : " << dt << "\n Desired_point: " << cv::Point2f((*desired_line)(0),(*desired_line)(1)) << " Desired Angle: " << atan((*desired_line)(2))/M_PI*180.0 << "\n  Observerd_pt: " << observed_pt << " Observed Angle: " << observed_angle/M_PI*180.0 << "\n ERROR (X): " << error_control(0) << " (theta) " << error_control(1)/M_PI*180 << "\n Delta: " << delta/M_PI*180 << " [degree] \n delta: " << delta << " [rad] "  << std::endl;

	//std::max(std::min(delta,(double)0.6),(double)-0.6);

	//return delta; // [rad]
	return 0.0f;
}

/**
 * @brief Send command to screen
 * 
 */
void LineProcessing::SendCommandEB2()
{
	int rate = 30;
	ros::Rate r(rate);

	mavros_msgs::OverrideRCIn msg_override;

	while (n->ok())
	{
		msg_override.channels[0] = erlerover_manager.getAngularVelocity();
		msg_override.channels[1] = 0;
		msg_override.channels[2] = erlerover_manager.getLinearVelocity();
		msg_override.channels[3] = 0;
		msg_override.channels[4] = 0;
		msg_override.channels[5] = 0;
		msg_override.channels[6] = 0;
		msg_override.channels[7] = 0;

		rc_override_pub.publish(msg_override);
		r.sleep();
	}
}

/**
* @brief ROS callback that return the image from the topic
* @param msg sensor_msgs::Image
*/

void LineProcessing::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		//std::cout<< " image callback... " <<std::endl;
		lineExtraction(msg);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
};
