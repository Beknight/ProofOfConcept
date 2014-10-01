#include "KalmanFilter.h"
#include <math.h>
#include "Util.h"
#include "time.h"
using namespace cv;
using namespace std;
#define X_POS 0 
#define Y_POS 1
#define Z_POS 2
#define X_VEL 3
#define Y_VEL 4
#define Z_VEL 5
namespace Thesis{
	KalmanFilter::KalmanFilter()
	{
		//x = the predicted 
		u_ = Mat(STATE_VECTOR_SIZE, 1, CV_64F, Scalar(0));
		covariance_ = Mat(STATE_VECTOR_SIZE, STATE_VECTOR_SIZE, CV_64F ,Scalar(0));
		angleConstX = (Camera::reso_width * 0.5) / tan(Util::convertToRadians(Camera::hor_fov*0.5)); // degrees per pixel 
		angleConstY = (Camera::reso_height * 0.5) / tan(Util::convertToRadians(Camera::vert_fov*0.5)); // degrees per pixel 
		timeLastUpdate = 0-0;
		// initialise the process noise matrix
		Q_Mono = Mat(2, 2, CV_64F, Scalar(0));
		Q_Stereo = Mat(3, 3, CV_64F, Scalar(0));
		R_ = Mat(STATE_VECTOR_SIZE, STATE_VECTOR_SIZE, CV_64F, Scalar(0));
		Q_Mono.at<double>(0, 0) = 10 * 10;
		Q_Mono.at<double>(1, 1) = 15 * 15;
		double measurementNoise = 15 * 15;
		Q_Stereo.at<double>(0, 0) = measurementNoise;
		Q_Stereo.at<double>(1, 1) = measurementNoise;
		Q_Stereo.at<double>(2, 2) = measurementNoise;
		double updateNoise = 300 * 300; // 300 ^2
		R_.at<double>(3, 3) = updateNoise;
		R_.at<double>(4, 4) = updateNoise;
		R_.at<double>(5, 5) = updateNoise;
	}	

	KalmanFilter::~KalmanFilter()
	{
	}

	void KalmanFilter::initialise(CoordinateReal position){
		//update the x_ with the current location of the matrix
		u_.at<double>(X_POS, 0) = position.x();
		u_.at<double>(Y_POS, 0) = position.y();
		u_.at<double>(Z_POS, 0) = position.z();
		// update the time
		timeLastUpdate = double (clock()) / CLOCKS_PER_SEC;
		//printTimeLastUpdate();
		writeDataToFile(false);
	}

	CoordinateReal KalmanFilter::getCurrentLocation(){
		// convert the current state vector into a Coordinate real object
		CoordinateReal location;
		double x = u_.at<double>(X_POS, 0);
		double y = u_.at<double>(Y_POS, 0);
		double z = u_.at<double>(Z_POS, 0);
		location.setX(x);
		location.setY(y);
		location.setZ(z);

		return location;
	}

	cv::Mat KalmanFilter::buildObsMat(int rows, CoordinateReal loc){
		Mat matrix(rows, 1, CV_64F, Scalar(0));
		matrix.at<double>(X_POS, 0) = loc.x();
		matrix.at<double>(Y_POS, 0) = loc.y();
		if (rows == 3){
			matrix.at<double>(Z_POS, 0) = loc.z();
		}
		return matrix;
	}

	void KalmanFilter::updateCovariance(Mat k){
		//covariance = (I - KH)* covariance
		//Mat innerBrackets = ;
	}

	void KalmanFilter::observation(CoordinateReal pixelCo, Camera camera){
		// get the expect obs

		//get the innovation

		//kalman gain 
		
		//update the step
	}

	void KalmanFilter::stereoObservation(CoordinateReal obs){
		// expected obs
		Mat zDash = expectedStereoObservation();
		Mat observation = buildObsMat(3, obs);
		//innovation
		Mat innovation = observation - zDash;
		//kalman gain
		
		//update step
	}

	cv::Mat KalmanFilter::getInnovation(Mat obs, Mat expectedObs){
		//innovation = obs - expObs
		cv::Mat innovation = obs - expectedObs;
		return innovation;
	}

	cv::Mat KalmanFilter::kalmanGain(Mat H, Mat Q){
		Mat inversePart = (H * covariance_ * H.t() + Q);	
		Mat kalmanGain = covariance_ * H * inversePart.inv();
		return kalmanGain;
	}

	CoordinateReal KalmanFilter::expectedLocObs(Camera camera){
		CoordinateReal realLoc;
		Mat loc = expectedObservation(camera);
		double x = loc.at<double>(0, 0);
		double y = loc.at<double>(1, 0);
		realLoc.setX(x);
		realLoc.setY(y);
		return realLoc;
	}
	void KalmanFilter::setStereoHJac(){
		H_Jacobian = Mat::eye(3, 3, CV_64F);
	}
	Mat KalmanFilter::expectedStereoObservation(){
		// the expected observation 
		Mat expectedObs = (Mat_<double>(3, 1) << 0, 0, 0);
		expectedObs.at<double>(X_POS, 0) = u_.at<double>(X_POS, 0);
		expectedObs.at<double>(Y_POS, 0) = u_.at<double>(Y_POS, 0);
		expectedObs.at<double>(Z_POS, 0) = u_.at<double>(Z_POS, 0);
		// set the H jacobian for the stereo Observation
		setStereoHJac();
		return expectedObs;
	}

	Mat KalmanFilter::expectedObservation(Camera camera){
		Mat expectedObs = (Mat_<double>(2, 1) << 0, 0);
		Mat hJacobian = Mat(2, 3, CV_64F, Scalar(0));

		CoordinateReal cameraLoc(camera.location().x(), camera.location().y(), camera.location().z());
		// distance is delx + dely +delz all squared
		double angleX = 0;
		double angleY = 0;
		double xLoc = 0;
		double yLoc = 0;
		double delX = u_.at<double>(0, 0) - cameraLoc.x();
		double delY = u_.at<double>(1, 0) - cameraLoc.y();
		double delZ = u_.at<double>(2, 0) - cameraLoc.z();
		double qRoot = delX * delX + delY * delY + delZ + delZ;
		qRoot = sqrt(qRoot);
		
		angleX = camera.yaw() - atan2(delZ, delX);
		angleY = camera.pitch() - atan2(delY, delZ);
		// angleX/FOV  * resolution width
		//this gives you the x-coordinate 
		xLoc = Camera::reso_width * 0.5 + angleConstX * tan(angleX);
		// angleY / fov * resoltuion height
		// this gives you the y-co-ordinate
		yLoc = Camera::reso_height * 0.5 + angleConstY * tan(angleY);
		//expected observation 
		expectedObs.at<double>(0, 0) = xLoc;
		expectedObs.at<double>(1, 0) = yLoc;
		// set the h jacobian
		setHJacobian(angleX, angleY, delX, delY, delZ);
		return expectedObs;
	} 

	void KalmanFilter::setHJacobian(double angleX, double angleY, double delX, double delY, double delZ){
		double zetaX = 0;
		double zetaZ = 0;
		double omegaY = 0;
		double omegaZ = 0;
		double zetaFirst = 0;
		double omegaFirst = 0;
		//zero the matrix
		H_Jacobian = Mat(2, 3, CV_64F, Scalar(0));
					//zeta * sec^2(gamma) * partial
		zetaFirst = angleConstX * (1 / cos(angleX)) * (1 / cos(angleX));
		omegaFirst = angleConstY * (1 / cos(angleY)) * (1 / cos(angleY));
		//the last part of the partial derives first line
		zetaX = H_JacPart(delZ, delX);
		zetaX = zetaFirst * zetaX;
		zetaZ = H_JacPart((delX * -1), delZ);
		zetaZ = zetaFirst * zetaZ;
		// the last part of teh partial derives second line
		omegaY = H_JacPart((delZ * -1), delY);
		omegaY = omegaFirst * omegaY;
		omegaZ = H_JacPart((delY), delZ);
		omegaZ = omegaFirst * omegaZ;
		//set up the matrix
		H_Jacobian.at<double>(0, 0) = zetaX;
		H_Jacobian.at<double>(0, 2) = zetaZ;
		H_Jacobian.at<double>(1, 1) = omegaY;
		H_Jacobian.at<double>(1, 2) = omegaZ;
		Util::printMatrix(H_Jacobian);
	}

	double KalmanFilter::H_JacPart(double top, double bot){
		double quotient = 0;
		//top / (bot^2 + top^2)
		quotient = top / (bot*bot + top*top);
		return quotient;
	}

	void KalmanFilter::printCurrentState(){
		//cout << "printCurrentState" << endl;
		for (int i = 0; i < u_.rows; i++){
			cout << u_.at<double>(i, 0) << endl;
		}
		// print the covariance matrix  
	}

	void KalmanFilter::predictState(){
		clock_t curTime = clock();
		double nowTime = double(curTime)/ CLOCKS_PER_SEC;
		double delT = 0;
		delT = nowTime - timeLastUpdate;
		// apply the motion model to the current state
		// only states 0-2 change the velocity we assume as 'constant' 
		//u'(t) = u(t-1) + vel(t)*delta time
		u_.at<double>(X_POS, 0) = u_.at<double>(X_POS, 0) + u_.at<double>(X_VEL,0) * delT;
		u_.at<double>(Y_POS, 0) = u_.at<double>(Y_POS, 0) + u_.at<double>(Y_VEL, 0) * delT;
		u_.at<double>(Z_POS, 0) = u_.at<double>(Z_POS, 0) + u_.at<double>(Z_VEL, 0) * delT;
		// velocity  parts do not get updated.
		//update the last run time
		writeDataToFile(true);
		// update the covariance
		cv::Mat G = motionModelJacobian(delT);
		Util::printMatrix(G);
		covariance_ = G * covariance_ * G.t() + R_;
		//Util::printMatrix(covariance_);
		//printTimeLastUpdate();
		timeLastUpdate = nowTime;

	}

	cv::Mat KalmanFilter::motionModelJacobian(double deltaTime){
		// identity matrix + the delta t's
		Mat G = Mat::eye(STATE_VECTOR_SIZE,STATE_VECTOR_SIZE, CV_64F);
		// add the delta t's to the respective positions
		G.at<double>(0, 3) = deltaTime;
		G.at<double>(1, 4) = deltaTime;
		G.at<double>(2, 5) = deltaTime;
		return G;
	}

	void KalmanFilter::writeDataToFile(bool isPrediction){
		// write the current string out	
		string outPutString;
		string x	= to_string(u_.at<double>(X_POS, 0));
		string xVel = to_string(u_.at<double>(X_VEL, 0));
		string y	= to_string(u_.at<double>(Y_POS, 0));
		string yVel = to_string(u_.at<double>(Y_VEL, 0));
		string z	= to_string(u_.at<double>(Z_POS, 0));
		string zVel = to_string(u_.at<double>(Z_VEL, 0));
		string time = to_string(timeLastUpdate);
		string colon = " : ";
		outPutString = x + colon + y + colon + z + colon + xVel + colon + yVel + colon + zVel + colon + time + "\n";
		dataFile_ << outPutString;
	}

	void KalmanFilter::openFile(){
		dataFile_.open("../../../../ThesisImages/data.txt");
	}

	void KalmanFilter::closeFile(){
		dataFile_.close();
	}
}