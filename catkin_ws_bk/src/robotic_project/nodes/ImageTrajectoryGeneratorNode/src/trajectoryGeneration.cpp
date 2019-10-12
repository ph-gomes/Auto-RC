#include "trajectoryGeneration/trajectoryGeneration.h"
/*

std::istream& operator>>(std::istream& is, cv::Point2d& coordinates)
{
	//int x,y;    	
	//is >> y >> x;
	
	is >> std::stod(coordinates.y) >> std::stod(coordinates.x);
	
	//coordinates.y = y; 
	//coordinates.x = x;

    return is;
}

*/

std::vector<std::string> split_string(const std::string& str,
                                      const std::string& delimiter)
{
    std::vector<std::string> strings;

    std::string::size_type pos = 0;
    std::string::size_type prev = 0;
    while ((pos = str.find(delimiter, prev)) != std::string::npos)
    {
        strings.push_back(str.substr(prev, pos - prev));
        prev = pos + 1;
    }

    // To get the last substring (or only, if delimiter is not found)
    strings.push_back(str.substr(prev));

    return strings;
}



trajectoryGeneration::trajectoryGeneration(ros::NodeHandle *rosNode_): rosNode(rosNode_)
{
     	g_window_name = std::string("trajectoryGeneration");
  	cv::namedWindow(g_window_name);
	
	cv::setMouseCallback(g_window_name, onMouse, this);

	std::string topic = rosNode->resolveName("image");

	std::string file = rosNode->resolveName("pathfile");
	std::cout << "Path file: " << file << std::endl;
	if(file.compare("/pathfile") != 0)
		trajectoryReader(file);

	image_transport::ImageTransport it(*rosNode);  	
  	image_sub = it.subscribe(topic, 1,boost::bind(&trajectoryGeneration::image_receive, this, _1));

	poseArray_pub = rosNode->advertise<geometry_msgs::PoseArray>("/trajectoryGeneration/poses", 1);
		
	// Start the OpenCV window thread so we don't have to waitKey() somewhere
  	cv::startWindowThread();
}





void trajectoryGeneration::trajectoryReader(std::string fileName)
{

	points.clear();   

	/*
    	std::ifstream ifs(fileName.c_str());

	std::cout << "Path file: " << fileName << " reading data... " << std::endl;

    	if (ifs) {
        	std::copy(std::istream_iterator<cv::Point2d>(ifs), 
                std::istream_iterator<cv::Point2d>(),
                std::back_inserter(points));
    	}
    	else {
        	std::cerr << "Couldn't open " << fileName << " for reading\n";
    	}


	

	std::cout << "\n  :: finished path reader ::  PATH size: " << points.size() << std::endl;


*/



	fileName += ".txt";
	std::ifstream file(fileName.c_str());
    	std::string str; 
	std::cout << "Path file: " << fileName << " reading data... " << std::endl;
    	while (std::getline(file, str))
    	{
        	// Process str
		std::vector<std::string> coordinates = split_string((const std::string) str,"	");

		double y = strtod(coordinates[0].c_str(), NULL); 
		double x = strtod(coordinates[1].c_str(), NULL); 

		//std::cout << "string: " << str <<  " x: " << x << " y: " << y << std::endl;
		std::cout <<  " x: " << x << " y: " << y << std::endl;

		points.push_back(cv::Point2d(x,y));
    	}

	std::cout << "\n  :: finished path reader ::  PATH size: " << points.size() << std::endl;
	
}


trajectoryGeneration::~trajectoryGeneration()
{
	cv::destroyWindow(g_window_name);
}


void trajectoryGeneration::image_receive(const sensor_msgs::ImageConstPtr& msg)
{
  	boost::mutex::scoped_lock lock(g_image_mutex);

  	try
    	{
      		g_last_image = cv_bridge::toCvShare(msg, "bgr8")->image;
    	} catch(cv_bridge::Exception)
    	{
     		ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
      		//return false;
    	}

  	if (!g_last_image.empty()) 
	{
    		const cv::Mat &image = g_last_image;
    		image_processing(image);
    
  	}
}

void trajectoryGeneration::image_processing(const cv::Mat &image)
{
	
	cv::Mat gray,src;	
	src = image;

	
  	/// Draw the circles detected
  	for( size_t i = 0; i < points.size(); i++ )
  	{      		
      		cv::circle( src, points[i], 3, cv::Scalar(0,255,0), -1, 8, 0 );      		
   	}
	
	imageShow((const cv::Mat) src);

}

void trajectoryGeneration::imageShow(const cv::Mat &image)
{
	//ROS_INFO("image show");
	cv::imshow(g_window_name, image);
	cv::waitKey(3);

}

void trajectoryGeneration::onMouse(int event, int x, int y, int flag)
{
	
	//ROS_INFO(" On Mouse... %d  %d %d ",flag, cv::EVENT_FLAG_CTRLKEY, cv::EVENT_FLAG_LBUTTON);
	if ( (flag == cv::EVENT_FLAG_LBUTTON && event == cv::EVENT_MOUSEMOVE) || (flag == 33 && event == cv::EVENT_MOUSEMOVE)) 	
	{
    		ROS_INFO("Mouse CallBack, Size: %d - point.x: %d  point.y:%d ",(int)points.size(),x,y);
		points.push_back(cv::Point2d(x,y));
		    		
  	}
	if (event == cv::EVENT_RBUTTONDOWN) {
		ROS_INFO("Cleanning Path... !!! " );
		points.clear();
		
  	}

	if ( (flag == cv::EVENT_FLAG_CTRLKEY || flag == 41)  && points.size() > 0)/////
	{
		ROS_INFO("Sending Trajectory...");
		geometry_msgs::PoseStamped pose;
		geometry_msgs::Quaternion orientation;

		path.poses.clear(); // Clear last block perception result
		path.header.stamp = ros::Time::now();
		path.header.frame_id = "/vehicle_pose";
		
		for(size_t i = 1; i<points.size(); ++i)
		{
			//adding pose to pose array
    			pose.pose.position.x = points[i].x;
    			pose.pose.position.y = points[i].y;
    			//pose.pose.position.z = 0.0;
			
			//TODO Extracts pose orientation from two consecutives points.
			pose.pose.orientation = orientation;
			double v_x = points[i].x - points[i-1].x;
			double v_y = points[i].y - points[i-1].y;
			pose.pose.position.z = anguloCalculate(1.0, 0.0, v_x,v_y);
			
			
			// Add points to trajectory...
    			path.poses.push_back(pose.pose);
			
			ROS_INFO("Path --> x: %f y: %f theta: %f ",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);

		}

		poseArray_pub.publish(path);
  		//ROS_INFO("Trajectory Points size: %i", path.poses.size()); 
	}	

}

void trajectoryGeneration::onMouse(int event, int x, int y, int flag, void* userdata)
{
	trajectoryGeneration* trajectory = reinterpret_cast<trajectoryGeneration*>(userdata);
	trajectory->onMouse(event, x, y, flag);
}

double trajectoryGeneration::anguloCalculate(double x0, double y0, double x1, double y1) //////

{	
	double dot = (x0*x1)+(y0*y1); 
	double det = (x0*y1)-(y0*x1);
	double angle;

	angle = atan2(det, dot);
	angle = (180/PI)*angle;

        //angle = (y1<0) ? angle+360.0 : angle ;
	
	return angle;
}
