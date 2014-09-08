#include "KalmanFilter.h"

using namespace cv;

KalmanFilter::KalmanFilter()
{
	//x = the predicted 
	x_ = Mat(3, 1, CV_32FC1);

}


KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::initialise(CoordinateReal position){
	//update the x_ with the current location of the matrix
}

Mat KalmanFilter::expectedObservation(Camera camera){
	Mat observation(3,1,CV_32FC1);
	return observation;
}

void KalmanFilter::execute(){
	while (1){
		//grab an image 

		//predict state

		//log prediction

		//update 

		//log update
	}
}