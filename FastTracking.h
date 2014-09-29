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
		int iLowH		= 14;
		int iHighH		= 179;
		//S================
		int iLowS		= 131;
		int iHighS		= 255;
		//V================
		int iLowV		= 57;
		int iHighV		= 255;
	};
}

