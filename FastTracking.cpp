#include "FastTracking.h"

#define MAX_VALUE 255
#define BLUR_VAL 10
FastTracking::FastTracking(int sensitivity)
{
	thresholdValue_ = sensitivity;
}

CoordinateReal FastTracking::findObject(Mat first, Mat second){
	CoordinateReal coord;
	Mat greyOne, greyTwo;
	Mat difference;
	Mat threshImage;
	// gray scale both of them 
	cvtColor(first,greyOne,COLOR_BGR2GRAY);
	cvtColor(second, greyTwo, COLOR_BGR2GRAY);
	// get the asbolue difference of the images
	absdiff(greyTwo,greyOne,difference);
	//threshold the image
	threshold(difference,threshImage,thresholdValue_,MAX_VALUE,THRESH_BINARY);
	//blur  the images
	blur(threshImage, threshImage, cv::Size(BLUR_VAL,BLUR_VAL));
	//threshold again
	threshold(threshImage, threshImage, thresholdValue_, MAX_VALUE, THRESH_BINARY);

	imshow("threshold",threshImage);
	return coord;
}

CoordinateReal searchForMovement(Mat threshImage){
	bool found = false;
	Mat throwAway;
	threshImage.copyTo(threshImage);
	vector<vector<Point>> contours;
	vector<Vec4i> hiearchy;
	findContours(throwAway, contours, hiearchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	found = (contours.size() > 0);
	if (found){

	}
}

FastTracking::~FastTracking()
{
}
