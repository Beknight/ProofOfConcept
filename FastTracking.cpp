#include "FastTracking.h"

#define MAX_VALUE 255
#define BLUR_VAL 10
FastTracking::FastTracking(int sensitivity)
{
	thresholdValue_ = sensitivity;
}

CoordinateReal FastTracking::findObject(Mat first, Mat second){
	CoordinateReal coord;
	Point location;
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
	location = searchForMovement(threshImage);
	coord.setX(location.x);
	coord.setY(location.y);
	imshow("threshold",threshImage);
	return coord;
}

Point FastTracking::searchForMovement(Mat threshImage){
	bool found = false;
	Mat throwAway;
	Point objectLocation;
	Rect foundObject;
	threshImage.copyTo(throwAway);
	vector<vector<Point>> contours;
	vector<Vec4i> hiearchy;
	findContours(throwAway, contours, hiearchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	found = (contours.size() > 0);
	// check that countours were found 
	if (found){
		vector < vector<Point> > largestContour;
		largestContour.push_back(contours.at(contours.size() - 1));
		// now we have found it we will draw a dot in the middle
		foundObject = boundingRect(largestContour.at(0));
		int xPos = foundObject.x + foundObject.width / 2;
		int yPos = foundObject.y + foundObject.height / 2;
		objectLocation.x = xPos;
		objectLocation.y = yPos;
	}
	return objectLocation;


}

FastTracking::~FastTracking()
{
}
