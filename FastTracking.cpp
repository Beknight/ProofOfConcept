#include "FastTracking.h"
#include "Util.h"
#include "Camera.h"
#define MAX_VALUE 255
#define BLUR_VAL 25
namespace Thesis{
	FastTracking::FastTracking(int sensitivity)
	{
		thresholdValue_ = sensitivity;
	}

	CoordinateReal FastTracking::findObject(Mat first, Mat second, CoordinateReal currentBelief, bool debug){
		CoordinateReal coord;
		Point location;
		Mat greyOne, greyTwo;
		Mat difference;
		Mat threshImage;
		// gray scale both of them 
		cvtColor(first, greyOne, COLOR_BGR2GRAY);
		cvtColor(second, greyTwo, COLOR_BGR2GRAY);
		// get the asbolue difference of the images
		absdiff(greyTwo, greyOne, difference);
		if (debug){
			imshow("difference", difference);
		}
		//threshold the image
		threshold(difference, threshImage, thresholdValue_, MAX_VALUE, THRESH_BINARY);
		//blur  the images
		if (debug){
			imshow("threshImage: ", threshImage);
		}
		blur(threshImage, threshImage, cv::Size(BLUR_VAL, BLUR_VAL));
		//threshold again
		threshold(threshImage, threshImage, thresholdValue_, MAX_VALUE, THRESH_BINARY);
		location = searchForMovement(threshImage, first, currentBelief);
		coord.setX(location.x);
		coord.setY(location.y);
		if (debug){
			imshow("threshold", threshImage);
		}
		return coord;
	}

	Point FastTracking::searchForMovement(Mat threshImage, Mat second, CoordinateReal currentBelief){
		bool found = false;
		Mat throwAway;
		Point objectLocation;
		Rect foundObject;
		threshImage.copyTo(throwAway);
		vector<vector<Point>> contours;
		vector<Vec4i> hiearchy;
		findContours(throwAway, contours, hiearchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
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
			// search colour using a reduced search space with colours
			Rect reducedSearch = Util::resizeRect(foundObject, second.rows, second.cols, 1.5);
			Mat subMatrix = second(reducedSearch);
			
			if (subMatrix.cols != 0 && subMatrix.rows != 0){
				//colour search 
				Mat throwSubMatrix = colorSearch(subMatrix);
				contours = vector<vector<Point>>();
				hiearchy = vector<Vec4i>();
				findContours(throwSubMatrix, contours, hiearchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
				if (contours.size() > 0){
					vector < vector<Point> > reducedLargeContour;
					reducedLargeContour.push_back(contours.at(contours.size() - 1));
					foundObject = boundingRect(reducedLargeContour.at(0));
					xPos = foundObject.x + foundObject.width / 2 + reducedSearch.x;
					yPos = foundObject.y + foundObject.height / 2 + +reducedSearch.y;
					objectLocation.x = (objectLocation.x + xPos)/2;
					objectLocation.y = (objectLocation.y + yPos)/2;
				}
			}
		} else if (!found){
			contours = vector<vector<Point>>();
			hiearchy = vector<Vec4i>();
			Rect smallRect = Util::getQuadrantRect(currentBelief);
			Mat temp = second(smallRect);
			imshow("quad", temp);
			Mat newThresh = colorSearch(temp);
			findContours(newThresh, contours, hiearchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			
			found = (contours.size() > 0);
			if (found){
				vector < vector<Point> > largestContour;
				largestContour.push_back(contours.at(contours.size() - 1));
				// now we have found it we will draw a dot in the middle
				foundObject = boundingRect(largestContour.at(0));
				int xPos = foundObject.x + foundObject.width / 2 + smallRect.x;
				int yPos = foundObject.y + foundObject.height / 2 + smallRect.y;
				objectLocation.x = xPos;
				objectLocation.y = yPos;
			}	
		} 
		
		return objectLocation;


	}

	bool FastTracking::checkForOcclusion(){
		bool isLost = false;
		// if the shape has dissapeared

		// check how likely it is to have stopped in the frame 
		return isLost;
	}

	Mat FastTracking::colorSearch(Mat second){
		cv::Mat hsvFrame;
		cv::Mat threshold;
		//check if we are searching the full space or the sub space 
		cvtColor(second, hsvFrame, COLOR_BGR2HSV);
		inRange(hsvFrame, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), threshold);
		//morphological opening (remove small objects from the foreground)
		/*erode(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));*/
				blur(threshold, threshold, cv::Size(20, 20));
				cv::threshold(threshold,threshold,60,255,THRESH_BINARY);
		return threshold;
	}

	FastTracking::~FastTracking()
	{
	}
}