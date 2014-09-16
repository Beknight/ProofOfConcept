#pragma once
#include <opencv\cv.h>
#include "LibraryHeaders.h"
#include "CoordinateReal.h"
using namespace cv;

class FastTracking
{
public:
	FastTracking(int thresholdValue );
	CoordinateReal findObject(Mat frist, Mat second);
	CoordinateReal searchForMovement(Mat threshImage);
	~FastTracking();
private: 
	int thresholdValue_;
	
};

