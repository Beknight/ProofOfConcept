#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "LogRecorder.h"
#include "Stereoscope.h"
#include "Camera.h"
#include <fstream>

namespace Thesis{
	using namespace std;
	class KalmanFilter
	{
	public:
		static const int STATE_VECTOR_SIZE = 6;
		//setup
		KalmanFilter();
		~KalmanFilter();
		void initialise(CoordinateReal initialPosition);
		cv::Mat expectedMonoObservation(Camera camera);
		CoordinateReal expectedLocObs(Camera camera);
		cv::Mat expectedStereoObservation();
		void observation(CoordinateReal pixelCo, Camera camera);
		void stereoObservation(CoordinateReal state);
		
		CoordinateReal getCurrentLocation();
		cv::Mat getCurrentPrediction(){ return u_; };
		// kalman filter run
		void openFile();
		void closeFile();
		void predictState();
		void printCurrentState();
		void printTimeLastUpdate(){ cout << "lastUpdate: " << timeLastUpdate << endl; };
	private:
		ofstream dataFile_;
		ofstream obsFile_;
		// the angle constants that we need for 
		double angleConstX = 0;
		double angleConstY = 0;
		// observation 
		//the two required classes to help out 
		LogRecorder logRecorder_;
		double timeLastUpdate;
		// the four required kalman filter fields
		cv::Mat covariance_;
		cv::Mat Q_Mono;
		cv::Mat Q_Stereo;
		cv::Mat R_;
		cv::Mat u_;
		cv::Mat H_Jacobian;
		void writeObsToFile(Mat obs);
		//prediction steps
		cv::Mat predictState(cv::Mat controlInput);
		cv::Mat predictionCovariance();
		//update state
		cv::Mat kalmanGain(Mat jacobian, Mat obsNoise);
		cv::Mat getInnovation(Mat obs, Mat expectedObs);
		cv::Mat updateStep(Mat kalmanProduct);
		void updateCovariance(Mat K);
		void setStereoHJac();
		void setHJacobian(double angleX, double angleY, double delX, double delY, double delZ);
		double H_JacPart(double top, double bot);
		cv::Mat getCurrentState(){ return u_; }
		cv::Mat buildObsMat(int rows, CoordinateReal loc);
		cv::Mat motionModelJacobian(double deltaTime);
		void writeDataToFile(bool isPrediction);
	};
}

