#pragma once
#include "LibraryHeaders.h"
#include <string>
#include "CoordinateReal.h"
using namespace cv;

class Camera
{
	
public:
	static const int reso_height	= 480;
	static const int reso_width		= 864;
	static const int hor_fov		= 50; //deg
	static const int vert_fov		= 30; //deg
	Camera(){};
	Camera(int cameraNumber);
	Camera(int camearaNumber, float x, float y, float z, double pitch, double yaw);
	~Camera();
	//methods
	CoordinateReal location() { return location_; }
	double yaw() { return yaw_; }
	double pitch() { return pitch_;  }
	Mat grabFrame(); //grab the current frame from the webcam 
	void writeLatestImageToFile(string filename);

private:
	bool isSaving_;
	int cameraId_;
	CvCapture* capture_;
	Mat curFrame_;
	CoordinateReal location_;
	double yaw_;//top view angle from the x axis defined by camera zero
	double pitch_;//side view angle form the x axis
	
};

