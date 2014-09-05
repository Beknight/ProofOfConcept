#pragma once
#include "LibraryHeaders.h"
#include <string>
#include "CoordinateReal.h"
using namespace cv;

class Camera
{
	
public:
	static const int reso_height	= 640;
	static const int reso_width		= 1280;
	static const int hor_fov		= 50;
	Camera(int cameraNumber);
	~Camera();
	//methods
	Mat grabFrame(); //grab the current frame from the webcam 
	void writeLatestImageToFile(string filename);

private:
	bool isSaving_;
	int cameraId_;
	CvCapture* capture_;
	Mat curFrame_;
	CoordinateReal location;
	double angleAlpha;//top view angle from the x axis defined by camera zero
	double angleBeta;//side view angle form the x axis
	
};

