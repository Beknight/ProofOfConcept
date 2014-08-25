#pragma once
#include "LibraryHeaders.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "CoordinateReal.h"
#include <vector>
using namespace cv;
class Stereoscope
{
public:
	Stereoscope();
	double getDistance(CoordinateReal leftImage, CoordinateReal rightImage);
	
	//find circle
	~Stereoscope();
private:
	const int distanceApart_ = 310; //mm
	const double halfWidthBall_ = 48.2; //mm
	double halfPixelWidth_;
	double halfFOV_;
	double distance_;
	CoordinateReal currentLocation_;
	double convertToRadians(double degrees);
	void calculateX(double theta, double distance);
	void calculateY(double alpha, double distance);
	void calculateZ(double theta, double distance);
	
};

