#pragma once
#include "LibraryHeaders.h"
#include <string>
using namespace cv;

class Camera
{
	
public:
	static const int reso_height = 720;
	static const int reso_width = 1280;
	static const int hor_fov = 50;
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
};

