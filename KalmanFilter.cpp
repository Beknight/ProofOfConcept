#include "KalmanFilter.h"
#include <math.h>
using namespace cv;
using namespace std;
KalmanFilter::KalmanFilter()
{
	//x = the predicted 
	x_ = (Mat_<double>(3,1) << 0,0,0);

}


KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::initialise(CoordinateReal position){
	//update the x_ with the current location of the matrix
	x_.at<double>(0, 0) = position.x();
	x_.at<double>(1, 0) = position.y();
	x_.at<double>(2, 0) = position.z();
}

Mat KalmanFilter::expectedObservation(Camera camera){
	Mat expectedObs = (Mat_<double>(3, 1) << 0, 0, 0);
	CoordinateReal cameraLoc = camera.location();
	// distance is delx + dely +delz all squared
	double angleX = 0;
	double angleY = 0;
	double delX = x_.at<double>(0, 0) - cameraLoc.x();
	double delY = x_.at<double>(1, 0) - cameraLoc.y();
	double delZ = x_.at<double>(2, 0) - cameraLoc.z();
	double qRoot = delX * delX + delY * delY + delZ + delZ;
	qRoot = sqrt(qRoot);
	expectedObs.at<double>(1, 0) = qRoot;
	angleX = atan2(delZ,delX) -	camera.yaw();
	angleY = atan2(delY, delZ) - camera.pitch();
	return expectedObs;
}

void KalmanFilter::printCurrentState(){
	cout << "printCurrentState" << endl;
	for (int i = 0; i < x_.rows; i++){
		cout << x_.at<double>(i, 0) << endl;
	}
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