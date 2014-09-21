#include "Camera.h"
#include "Util.h"


Camera::Camera(int cameraNumber	)
{
	cameraId_ = cameraNumber;
	capture_ = cvCaptureFromCAM(cameraId_);
	isSaving_ = false;
	// set the camera settings 
	cvSetCaptureProperty(capture_, CV_CAP_PROP_FRAME_HEIGHT, reso_height);
	cvSetCaptureProperty(capture_, CV_CAP_PROP_FRAME_WIDTH, reso_width);
}

Camera::Camera(int cameraNumber, float x, float y, float z, double pitch, double yaw){
	cameraId_ = cameraNumber;
	capture_ = cvCaptureFromCAM(cameraId_);
	isSaving_ = false;
	// set the camera settings 
	cvSetCaptureProperty(capture_, CV_CAP_PROP_FRAME_HEIGHT, reso_height);
	cvSetCaptureProperty(capture_, CV_CAP_PROP_FRAME_WIDTH, reso_width);
	location_ = CoordinateReal(x,y,z);
	yaw_ = Util::convertToRadians(yaw);
	pitch_ = Util::convertToRadians(pitch);
}

Camera::~Camera()
{
}

Mat Camera::grabFrame(){
	IplImage* color_img;
	color_img = cvQueryFrame(capture_); // get frame
	curFrame_ = Mat(color_img);
	return curFrame_;
}

void Camera::writeLatestImageToFile(string fileName){
	cv::FileStorage file(fileName, cv::FileStorage::WRITE);
	file << curFrame_;
}