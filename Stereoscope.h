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
	CoordinateReal getLocation(CoordinateReal leftImage, CoordinateReal rightImage);
	//find circle
	~Stereoscope();
private:
	const int distanceApart_	= 250; //mm
	double halfPixelWidth_;
	double halfFOV_; 
	float vertFov_ = 30;
	double distance_;
	CoordinateReal currentLocation_;
	void narrowField();
	double convertToRadians(double degrees);
	double calculateX(double distance, double tanOmegaLeft, double tanOmegaRight);
	float calculateY(CoordinateReal leftImage,CoordinateReal rightImage, double distance, double x);
	void calculateZ(double theta, double distance);

};

