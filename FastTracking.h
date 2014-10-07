#pragma once
#include <opencv\cv.h>
#include "LibraryHeaders.h"
#include "CoordinateReal.h"
using namespace cv;
namespace Thesis{
	class FastTracking
	{
	public:
		FastTracking(int thresholdValue);
		CoordinateReal findObject(Mat frist, Mat second, CoordinateReal currentBelief, bool debug);
		Point searchForMovement(Mat threshImage, Mat second, CoordinateReal currentBelief);
		Mat colorSearch(Mat second);

		~FastTracking();
	private:
		int thresholdValue_;
		bool checkForOcclusion();
		bool velocityCheck();
		bool colourCheck();
		bool thresholdCheck(); // (?) we can threshold the image with the colours ? 
		// our hsv values
		//H================
		int iLowH		= 155; // outdoor 155 / 179
		int iHighH		= 179;
		//S================
		int iLowS		= 80;   //night time 80 // day time 131  // outdoor 75 / 255
		int iHighS		= 255;
		//V================
		int iLowV		= 51; //night time 70 // day time 57 // outdoor 0 
		int iHighV		= 255;
	};
}

