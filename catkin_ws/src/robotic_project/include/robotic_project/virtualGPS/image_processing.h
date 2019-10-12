#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <sstream>
#include <string>
#include <iostream>
#include <vector>

#include "Object.h"


class ImageProcessing
{
public:
    	    
    	//initial min and max HSV filter values.
	//these will be changed using trackbars
	int H_MIN;
	int H_MAX;
	int S_MIN;
	int S_MAX;
	int V_MIN;
	int V_MAX;
	

	//The following for canny edge detec
	Mat dst, detected_edges;
	Mat src, src_gray;
	int edgeThresh;
	int lowThreshold;
	int max_lowThreshold;
	int ratio;
	int kernel_size;
	
	
public:
    
	ImageProcessing();
	~ImageProcessing();
	void createTrackbars();
	void drawObject(vector<Object> theObjects,Mat &frame, Mat &temp, vector< vector<Point> > contours, vector<Vec4i> hierarchy);
	void drawObject(vector<Object> theObjects,Mat &frame);
	void morphOps(Mat &thresh);
	void trackFilteredObject(Mat threshold,Mat HSV, Mat &cameraFeed);
	bool trackFilteredObject(Object theObject,Mat threshold,Mat HSV, Mat &cameraFeed, vector<Object> &objects);
	
	//void run(Mat cameraFeed);
	//static void on_trackbar( int newValue, void* object);
    	//void showValue( int newValue);
};

#endif // GPS_H
