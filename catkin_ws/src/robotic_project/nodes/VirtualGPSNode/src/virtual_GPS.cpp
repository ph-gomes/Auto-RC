#include "virtualGPS/virtual_GPS.h"

virtualGPS::virtualGPS(ros::NodeHandle *rosNode_): rosNode(rosNode_)
{
     	g_window_name = std::string("Virtual GPS Node");
  	cv::namedWindow(g_window_name);


	//a_window_name = std::string("Imagem binarizada");
	windowName = std::string("Original Image");
  	cv::namedWindow(windowName);

	window_name = std::string("Edge Map");
  	cv::namedWindow(window_name);

	windowName1 = std::string("HSV Image");
  	cv::namedWindow(windowName1);

	windowName2 = std::string("Thresholded Image");
  	cv::namedWindow(windowName2);

	windowName3 = std::string("After Morphological Operations");
  	cv::namedWindow(windowName3);
	

	//Start the OpenCV window thread so we don't have to waitKey() somewhere
  	cv::startWindowThread();


	std::string topic = rosNode->resolveName("image");

	image_transport::ImageTransport it(*rosNode);  	
  	image_sub = it.subscribe(topic, 1,boost::bind(&virtualGPS::image_receive, this, _1));

	pose2D_pub = rosNode->advertise<geometry_msgs::Pose2D>("/virtualGPS/pose2D", 1);
		
	pose.x = prev_pose.x = 0.0;
	pose.y = prev_pose.y = 0.0;
	pose.theta = prev_pose.theta = 0.0;
	
	rho = 0.001;
	count = 0;
	
	roi.x = roi.y = 0;
	tile =400;

	

	//KF = new cv::KalmanFilter(4, 2, 0);
	KF = new cv::KalmanFilter(6, 3, 0);

	// intialization of KF...
	//KF->transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
	KF->transitionMatrix = *(cv::Mat_<float>(6, 6) << 1,0,1,0,1,0,   0,1,0,1,0,1,  0,0,1,0,0,0,  0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);
	
	measurement = new cv::Mat_<float> (3,1);
	measurement->setTo(cv::Scalar(0));
	KF->statePre.at<float>(0) = 0; //x
	KF->statePre.at<float>(1) = 0; //y
	KF->statePre.at<float>(2) = 0; //theta
	//KF->measurementMatrix= *(cv::Mat_<float>(2, 4) << 1,0,0,0,   0,1,0,0);
	KF->measurementMatrix= *(cv::Mat_<float>(3, 6) << 1,0,0,0,0,0,   0,1,0,0,0,0, 0,0,1,0,0,0);

	//KF->measurementNoiseCov= *(cv::Mat_<float>(2, 2) << 0.1,0,   0, 0.1);
	KF->measurementNoiseCov= *(cv::Mat_<float>(3, 3) << 0.1,0,0,   0, 0.1,0, 0,0,0.4);

	cv::setIdentity(KF->processNoiseCov, cv::Scalar::all(1e-4));
	   //cv::setIdentity(KF->measurementNoiseCov, cv::Scalar::all(0.5));
	cv::setIdentity(KF->errorCovPost, cv::Scalar::all(.1));


}

virtualGPS::~virtualGPS()
{
	delete (KF);
	delete (measurement);

	cv::destroyWindow(g_window_name);

	cv::destroyWindow(windowName);
	cv::destroyWindow(window_name);

}


void virtualGPS::image_receive(const sensor_msgs::ImageConstPtr& msg)
{
	
  	boost::mutex::scoped_lock lock(g_image_mutex);
	
		
	cv::Mat image;
  	try
    	{
      		image = cv_bridge::toCvShare(msg, "bgr8")->image;
    	} catch(cv_bridge::Exception)
    	{
     		ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
      		//return false;
    	}
	
	count++;
	rho = (count < 60) ? 0.5:0.001;
  	if (!mean.empty()) 
	{
    		//const cv::Mat &image = g_last_image;
    		image_processing((const cv::Mat) image);
    
  	} else 
	{
		//mean = image; 
		cv::Mat imgHSV;
		cv::cvtColor(image,imgHSV,COLOR_BGR2HSV);
		cv::split(imgHSV,mean);

		cv::equalizeHist(mean[0], mean[0]); //equalize histogram on the 1st channel (Y)
		//background_Gaussian(channels[0], mean[0], variance[0]);       	
		//cv::imshow(windowName3, mean[0] );

		//cv::equalizeHist(mean[1], mean[1]); //equalize histogram on the 1st channel (Y)
		//background_Gaussian(channels[1], mean[1]);

	       	//cv::equalizeHist(mean[2], mean[2]); //equalize histogram on the 1st channel (Y)
		//background_Gaussian(channels[2], mean[2]);

	       	//cv::merge(mean,image); //merge 3 channels including the modified 1st channel into one image
		//cv::cvtColor( image, mean[0], CV_BGR2GRAY);

		//mean = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);
		//variance.push_back(cv::Mat::ones(image.rows, image.cols, CV_32FC1 )*0.5);
	}
}

void virtualGPS::background_simples(const cv::Mat &image, cv::Mat &mean_, cv::Mat &variance_, cv::Mat &img_bw)
{
	float k = 50.0;

	
	// https://en.wikipedia.org/wiki/Background_subtraction
	

	//std::cout << "Image: " << count << " rho: " << rho << std::endl; //" mean_ witdh : " << mean_.cols << " mean_ height " << mean_.rows << std::endl;

	mean_ = ( (rho * image) + ((1.0 - rho) * mean_) ) ;
	
	cv::Mat diff = abs(image - mean_);
	
	cv::Mat dst;

	//cv::medianBlur(diff, diff, 3);

	img_bw = diff > k;		
	//cv::threshold(diff, img_bw, 0, 255, CV_THRESH_OTSU);


	
	//int morph_elem = 1; // MORPH_ELLIPSE
	//int morph_size = 10;
	//int morph_operator = 1; // MORPH_CLOSE

	//int operation = morph_operator + 2;

 	//cv::Mat element = cv::getStructuringElement( morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );

  	/// Apply the specified morphology operation
  	//cv::morphologyEx( dst, dst, operation, element );

	//cv::imshow(windowName3,  dst);

}

void virtualGPS::background_Gaussian(const cv::Mat &image, cv::Mat &mean_, cv::Mat &variance_, cv::Mat &img_bw)
{
	float k = 100.0;

		
	// https://en.wikipedia.org/wiki/Background_subtraction
	cv::Mat d = abs(image - mean_);
	//cv:Mat d = cv::Mat(image.rows, image.cols, CV_32FC1 );
	
	//std::cout << "0 d witdh : " << (d).cols << " d height " << (d).rows << std::endl;	
	//cv::Mat aux = d_aux.mul(d_aux); 
	//aux.convertTo(aux,CV_32FC1);	
	//cv::sqrt(aux,d);
	//std::cout << "00 d witdh : " << (d).cols << " d height " << (d).rows << std::endl;	
	d.convertTo(d,CV_32FC1);

	//std::cout << "Image: " << count << " rho: " << rho << " d witdh : " << (d).cols << " d height " << (d).rows << std::endl;

	mean_ = ( (rho * image) + ((1.0 - rho) * mean_) ) ;
	
	
	//cv::imshow(g_window_name, d );
	
	cv::Mat var_ = (1.0 - rho) * variance_.mul(variance_);
	cv::sqrt( (d * rho) + var_, variance_);
	
	//std::cout << "variance_ witdh : " << (variance_).cols << " variance_ height " << variance_.rows << std::endl;
	//cv::imshow(g_window_name, variance_ );

	d = abs(image - mean_);
	//aux = d_aux.mul(d_aux);
	//aux.convertTo(aux,CV_32FC1);
	//cv::sqrt(aux,d);
	d.convertTo(d,CV_32FC1);
	
	
	//std::cout << "d witdh : " << d.cols << " d height " << d.rows << std::endl;
	//std::cout << "variance witdh : " << variance.cols << " variance height " << variance.rows << std::endl;

	cv::Mat dst = (d / variance_);
	
	

	cv::normalize(dst, dst , 0, 255, NORM_MINMAX, CV_8U);
	//cv::divide(d,variance_,dst);	
	//dst = dst > k;
	
	//cv::imshow(windowName2,  dst);

	//cv::Mat img_bw;
	cv::threshold(dst, img_bw, 0, 255, CV_THRESH_OTSU);
	//dst.copyTo(img_bw);

	//std::cout << "dst type : " << dst.type() << " bw type " << img_bw.type() << std::endl;
	//std::cout << img_bw << std::endl;
	
	cv::imshow(windowName3,  img_bw);

	//(dst).convertTo(dst,CV_8UC3);
	//std::cout << "dst witdh : " << (dst).cols << " dst height " << (dst).rows << std::endl;

	//cv::imshow(windowName1, img_bw);
	//waitKey(30);
}
void virtualGPS::image_processing(const cv::Mat &image)
{
	//std::cout << "Image: " << count << " rho: " << rho << std::endl;

	//cv::imwrite("colorIMAGE.png", image);	
	//cv::Mat gray,src,graytre,graytrefil;	
	cv::Mat src = image;
	double v_x,v_y;

/*
	cv::vector<cv::Mat> channels; 
        cv::Mat img_hist_equalized;	
	cv::Mat imgHSV;

	//cv::Mat foreground = Mat::zeros(image.rows, image.cols, image.type());
        //cv::cvtColor(img, img_hist_equalized, CV_BGR2YCrCb); //change the color image from BGR to YCrCb format
	

	//cout << "roi.x: " << max(0,roi.x) << " roi.y: " << max(0,roi.y) << " width: " << min(image.cols, roi.x + roi.width) << " height: " << min(image.rows, roi.y + roi.height) << endl;
	//foreground = (roi.x < 0 && roi.y < 0 )? image: image(cv::Rect( max(0,roi.x) , max(0,roi.y), min(image.cols, roi.x + roi.width), min(image.rows, roi.y + roi.height) ) ).clone();
	//foreground = image;
	
	cv::cvtColor(image,imgHSV,COLOR_BGR2HSV);
	


	cv::split(imgHSV,channels);

        //cv::split(image,channels); //split the image into channels

    	cv::equalizeHist(channels[0], channels[0]); //equalize histogram on the 1st channel (Y)
	//background_Gaussian(channels[0], mean[0], variance[0]);       	
	//cv::imshow(windowName3, mean[0] );

	//cv::equalizeHist(channels[1], channels[1]); //equalize histogram on the 1st channel (Y)
	//background_Gaussian(channels[1], mean[1]);

       	//cv::equalizeHist(channels[2], channels[2]); //equalize histogram on the 1st channel (Y)
	//background_Gaussian(channels[2], mean[2]);

       	//cv::merge(channels,img_hist_equalized); //merge 3 channels including the modified 1st channel into one image

	//cv::Mat foreground;
	//background_Gaussian(image, foreground);
	
	//std::cout << "foreground witdh : " << foreground.cols << " foreground height " << foreground.rows << std::endl;

	//cv::imshow(g_window_name, img_hist_equalized);

	
	
	cv::Mat img_bwB,img_bwG,img_bwR;
	/// Convert it to gray
  	//cv::cvtColor( img_hist_equalized, gray, CV_BGR2GRAY);
	
	//cv::imshow(g_window_name, gray);

	//background_Gaussian(gray, mean[0], variance[0],img_bw);
	//background_simples(gray, mean[0], variance[0],img_bw);
	background_simples(channels[0], mean[0], variance[0],img_bwB);
	background_simples(channels[1], mean[1], variance[1],img_bwG);
	//background_simples(channels[2], mean[2], variance[2],img_bwR);
	
	//cv::imshow(windowName2, img_bwB);
	//cv::imshow(windowName3, img_bwG);
	//cv::imshow(g_window_name, img_bwR);
	
	//Mat m = Mat::ones(img_bwB.rows, img_bwB.cols, CV_8UC1) * 250;

	channels[0] = channels[0] & img_bwB; //channels[0].mul(img_bw);
	channels[1] = channels[1] & img_bwG; //channels[1].mul(img_bw);
	//channels[2] = m;//channels[2] & img_bwR; //channels[2].mul(img_bw);
	
	cv::merge(channels,imgHSV);
	
	cv::cvtColor(imgHSV,img_hist_equalized,COLOR_HSV2BGR);
	
	//cv::imwrite("color.png", img_hist_equalized);
	//cv::imshow(g_window_name, img_hist_equalized);

	//std::cout << "0 gray_ type: " << gray.type() << " gray_ depth " << gray.depth() << std::endl;
	//gray = gray.mul(img_bw);
	//gray.convertTo(gray,CV_8UC1);
	//img_bw.convertTo(gray,CV_8UC1 );
	
	//std::cout << "0 gray_ type: " << gray.type() << " gray_ depth " << gray.depth() << std::endl;

	//cv::cvtColor( img_bw, img_hist_equalized, CV_GRAY2BGR);
	//cv::cvtColor( img_hist_equalized, gray, CV_BGR2GRAY);
	//cv::imshow(windowName3,  img_hist_equalized);
*/	
	vector<Object> objects;
	
	if ( !(roi.x == 0 && roi.y == 0) ){

		//cout << "Estou dentro... roi: " << roi << endl;
		//cv::Mat foreground = Mat::zeros(roi.width, roi.height, image.type());
		//foreground = image(roi);
		//cout << "count " << count << " roi (0): " << roi << endl;	
		//cv::Rect aux= cv::Rect( max(0,roi.x) , max(0,roi.y), min(image.cols, roi.x + roi.width), min(image.rows, roi.y + roi.height) );		
		cv::Mat temp = image(roi);
		//temp.copyTo(foreground(roi));
		
		//foreground = image;
		
		cv::imshow(windowName, temp);

		colorProcessing(temp, objects);

		pose.x = roi.x; 
		pose.y = roi.y;
	}else
	{
		//cout << "Estou no outro... roi: " << roi << endl;
		//colorProcessing(img_hist_equalized, objects);
		colorProcessing(src, objects);
		pose.x = pose.y = 0;
	}




	//colorProcessing(img_hist_equalized, objects);

	cv::Mat prediction = KF->predict();
	
	if(objects.size()>0)
	{
				
		//cout<< "objects > 0 " << endl;

		if(objects.size() == 1)
		{
			cout<< "objects == 1 " << endl;
			cv::Point p = cv::Point(objects.at(0).getXPos(),objects.at(0).getYPos());
				
			pose.x += p.x;
			pose.y += p.y;
			
			cv::circle(src,cv::Point(pose.x,pose.y),10,cv::Scalar(255,0,255));

			v_x = pose.x - prev_pose.x;
		        v_y = pose.y - prev_pose.y;
		
			if(abs(v_x) > 1 || abs(v_y) > 1)
				pose.theta = this->anguloCalculate(1.0,0.0,v_x,v_y);
		
			(*measurement)(0) = pose.x;
			(*measurement)(1) = pose.y;
			(*measurement)(2) = pose.theta;

			// The update phase
			cv::Mat estimated = KF->correct(*measurement);

			pose.x = estimated.at<float>(0);
			pose.y = estimated.at<float>(1);
			pose.theta = estimated.at<float>(2);

		}
		if(objects.size() == 2)
		{
			//cout<< "objects == 2 " << endl;

			cv::Point pBlue = cv::Point(objects.at(0).getXPos(),objects.at(0).getYPos());
			cv::Point pRed = cv::Point(objects.at(1).getXPos(),objects.at(1).getYPos());
		
			float x = (pBlue.x + pRed.x)/2.0;
			float y = (pBlue.y + pRed.y)/2.0;

			//pose.x += /*roi.x +*/ objects.at(0).getXPos();
			//pose.y += /*roi.y + */ objects.at(0).getYPos();
		
			pose.x += x;
			pose.y += y;
		
			cv::circle(src,cv::Point(pose.x,pose.y),10,cv::Scalar(255,0,255));

			
			v_x = pBlue.x - pRed.x;
			v_y = pBlue.y - pRed.y;
			//cout << "v_x: " << v_x << " v_y: " << v_y << endl;		
			if( abs(v_x) > 1 || abs(v_y) > 1 )
			{
				pose.theta = this->anguloCalculate(1.0,0.0,v_x,v_y);

			}
			
			(*measurement)(0) = pose.x;
			(*measurement)(1) = pose.y;
			(*measurement)(2) = pose.theta;

			// The update phase
			cv::Mat estimated = KF->correct(*measurement);

			pose.x = estimated.at<float>(0);
			pose.y = estimated.at<float>(1);
			pose.theta = estimated.at<float>(2);
		

		}

		float fracao = tile * (1.0/2.0);
		roi.x = (int) std::min((float)image.cols, std::max((float) 0.0, (float) pose.x - fracao)) ;
		roi.y = (int) std::min((float)image.rows, std::max((float) 0.0, (float) pose.y - fracao));

	} else
	{
		//cout<< "objects < 0 " << endl;
		pose.x = prediction.at<float>(0);
		pose.y = prediction.at<float>(1);
		pose.theta = prediction.at<float>(2);

		v_x = pose.x - prev_pose.x;
		v_y = pose.y - prev_pose.y;
		
		if(abs(v_x) > 1 || abs(v_y) > 1 )
			pose.theta = this->anguloCalculate(1.0,0.0,v_x,v_y);
	
		cv::circle(src,cv::Point(pose.x,pose.y),10,cv::Scalar(255,0,0));

		roi.x = roi.y = 0;
	}
	
	//float weight = 0.6f;
	//pose.theta = ( prev_pose.theta * weight ) + ( pose.theta * (1 - weight)); 
	pose2D_pub.publish(pose);
		
	
				
	roi.width =   ((roi.x + tile) >= image.cols)? tile - ( (roi.x + tile) - image.cols) : tile; 
	roi.height=   ((roi.y + tile) >= image.rows)? tile - ( (roi.y + tile) - image.rows) : tile;
		
	drawDetection(src,objects, v_x, v_y );

	prev_pose = pose;
	//pose.x = pose.y = 0;
	cout << " Image: " << count << " Objects: "<< objects.size() << " :: prev_pose.x: " << prev_pose.x << " prev_pose.y: " << prev_pose.y << " prev_pose.theta: " << prev_pose.theta << endl;


	imageShow((const cv::Mat) src);

}

void virtualGPS::drawDetection(cv::Mat &src,vector<Object> &objects, double v_x, double v_y )
{

		int thickness = 2;
		int lineType = 8;
		line( src,
			cv::Point(pose.x,pose.y),
			cv::Point(pose.x + (v_x*1) , pose.y + (v_y*1) ),
			Scalar( 0, 255, 0 ),
			thickness,
			lineType );
		
		
		
		//cout << "count " << count << " roi (1): " << roi << endl;	
		rectangle(src, 
			  roi,
			  Scalar( 0, 0, 0 ));

		for(int i =0; i<objects.size(); i++){

			cv::circle(src,cv::Point(pose.x,pose.y),17,cv::Scalar(0,0,255));
			
		}


}


void virtualGPS::imageShow(const cv::Mat &image)
{

	cv::imshow(g_window_name, image);
	//cv::waitKey(3);

}

double virtualGPS::anguloCalculate(double x0, double y0, double x1, double y1)

{	
	
	double dot = (x0*x1)+(y0*y1); 
	double det = (x0*y1)-(y0*x1);
	double angle;

	angle = atan2(det, dot);
	angle = (180/PI)*angle;

        //angle = (y1<0) ? angle+360.0 : angle ;
	
	return angle;
	
	/*
	double scalar = (x0*x1)+(y0*y1);
	
	double norma = sqrt(x0*x0 + y0*y0 ) * sqrt(x1*x1 + y1*y1 );

	double angle = acos(scalar / norma) * 180.0 / PI;
	
	cout << "x0: " << x0 << " y0: " << y0 << " x1:" << x1 << " y1: " << y1 << "scalar: "<< scalar << "norma: " << norma << " angle: " << angle << endl;  
	return angle;
*/
}



void virtualGPS::colorProcessing(Mat cameraFeed, vector<Object> &objects)
{
	//cv::imwrite("colorRGB.png", cameraFeed);	
	//if we would like to calibrate our filter values, set to true.
	bool calibrationMode = false;

	//Matrix to store each frame of the webcam feed
	//Mat cameraFeed;
	Mat threshold;
	Mat HSV;

	

	if(calibrationMode){
		//create slider bars for HSV filtering
		createTrackbars();
	}
	//video capture object to acquire webcam feed
	//VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	//capture.open(0);
	//set height and width of capture frame
	//capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	//capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	//waitKey(1000);
	//while(1){
		//store image to matrix
		//capture.read(cameraFeed);

		imgProc.src = cameraFeed;

  		if( !imgProc.src.data )
  		{ return; }

		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);

		if(calibrationMode==true){

		//need to find the appropriate color range values
		// calibrationMode must be false

		//if in calibration mode, we track objects based on the HSV slider values.
			cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,Scalar(imgProc.H_MIN,imgProc.S_MIN,imgProc.V_MIN),Scalar(imgProc.H_MAX,imgProc.S_MAX,imgProc.V_MAX),threshold);
			imgProc.morphOps(threshold);
			imshow(windowName2,threshold);

		//the folowing for canny edge detec
			/// Create a matrix of the same type and size as src (for dst)
	  		imgProc.dst.create( imgProc.src.size(), imgProc.src.type() );
	  		/// Convert the image to grayscale
	  		cvtColor( imgProc.src, imgProc.src_gray, CV_BGR2GRAY );
	  		/// Create a window
	  		//namedWindow( window_name, CV_WINDOW_AUTOSIZE );
	  		/// Create a Trackbar for user to enter threshold
	  		createTrackbar( "Min Threshold:", window_name, &imgProc.lowThreshold, imgProc.max_lowThreshold);
			
			std::cout << "thresold: " << 	  imgProc.lowThreshold << std::endl;		
			/// Show the image
			imgProc.trackFilteredObject(threshold,HSV,cameraFeed);
			imshow(window_name,threshold);
		}
		else{
			//create some temp fruit objects so that
			//we can use their member functions/information
			Object blue("blue"); 
			//Object yellow("yellow");
			Object red("red");
			//Object green("green");

			//first find blue objects
			cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,blue.getHSVmin(),blue.getHSVmax(),threshold);
			imgProc.morphOps(threshold);
			imgProc.trackFilteredObject(blue,threshold,HSV,cameraFeed, objects);
			//then yellows
			//cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
			//inRange(HSV,yellow.getHSVmin(),yellow.getHSVmax(),threshold);
			//imgProc.morphOps(threshold);
			//imgProc.trackFilteredObject(yellow,threshold,HSV,cameraFeed);
			//then reds
			cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,red.getHSVmin(),red.getHSVmax(),threshold);
			imgProc.morphOps(threshold);
			imgProc.trackFilteredObject(red,threshold,HSV,cameraFeed, objects);
			//then greens
			//cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
			//inRange(HSV,green.getHSVmin(),green.getHSVmax(),threshold);
			//imgProc.morphOps(threshold);
			//imgProc.trackFilteredObject(green,threshold,HSV,cameraFeed);

		}
		//show frames
		//imshow(windowName2,threshold);

		//imshow(windowName,cameraFeed);

		
		//imshow(windowName1,HSV);
		//cv::imwrite("color.png", cameraFeed);		
		
		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		//waitKey(3);
	//}
	//return 0;

}


void virtualGPS::on_trackbar( int newValue, void* object)
{//This function gets called whenever a
	// trackbar position is changed
	virtualGPS* myClass = (virtualGPS*) object;
	myClass->showValue(newValue);
}


void virtualGPS::showValue( int newValue)
{
	std::cout << "new Value: " << newValue << std::endl;

}


void virtualGPS::createTrackbars(){
	//create window for trackbars
	//namedWindow(g_window_name,0);
	//create memory to store trackbar name on window
	//char TrackbarName[50];
	//sprintf( TrackbarName, "H_MIN", imgProc.H_MIN);
	//sprintf( TrackbarName, "H_MAX", imgProc.H_MAX);
	//sprintf( TrackbarName, "S_MIN", imgProc.S_MIN);
	//sprintf( TrackbarName, "S_MAX", imgProc.S_MAX);
	//sprintf( TrackbarName, "V_MIN", imgProc.V_MIN);
	//sprintf( TrackbarName, "V_MAX", imgProc.V_MAX);
	
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH),
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->
	createTrackbar( "H_MIN", g_window_name, &imgProc.H_MIN, imgProc.H_MAX, on_trackbar,this );
	createTrackbar( "H_MAX", g_window_name, &imgProc.H_MAX, imgProc.H_MAX, on_trackbar,this  );
	createTrackbar( "S_MIN", g_window_name, &imgProc.S_MIN, imgProc.S_MAX, on_trackbar,this  );
	createTrackbar( "S_MAX", g_window_name, &imgProc.S_MAX, imgProc.S_MAX, on_trackbar,this  );
	createTrackbar( "V_MIN", g_window_name, &imgProc.V_MIN, imgProc.V_MAX, on_trackbar,this  );
	createTrackbar( "V_MAX", g_window_name, &imgProc.V_MAX, imgProc.V_MAX, on_trackbar,this  );
}






