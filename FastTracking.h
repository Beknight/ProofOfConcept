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
		CoordinateReal findObject(Mat frist, Mat second, bool debug);
		Point searchForMovement(Mat threshImage);
		~FastTracking();
	private:
		int thresholdValue_;

	};
}

