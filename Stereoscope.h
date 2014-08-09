#pragma once
#include "LibraryHeaders.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
using namespace cv;
class Stereoscope
{
public:
	Stereoscope();
	double getDistance();
	//find circle
	~Stereoscope();
private:
	const int distanceApart_ = 310; //mm
	const double halfWidthBall_ = 48.2; //mm
	void displayImages();
	double halfPixelWidth_;
	double halfFOV_;
	double distance_;
	vector<Mat> targetImages_;
	vector<Mat> greyImages_;
	vector<Point> centers_;
	Mat blurPicture(Mat imageToBLur);
	Point findCircleCentre(Mat searchSpace, Mat original);
	double convertToRadians(double degrees);
	
};

