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
		//setup
		KalmanFilter();
		~KalmanFilter();
		void initialise(CoordinateReal initialPosition);
		cv::Mat expectedObservation(Camera camera);
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
		// the angle constants that we need for 
		double angleConstX = 0;
		double angleConstY = 0;
		// observation 
		//the two required classes to help out 
		LogRecorder logRecorder_;
		double timeLastUpdate;
		// the four required kalman filter fields
		cv::Mat covariance_;
		cv::Mat q_;
		cv::Mat r_;
		cv::Mat u_;
		//prediction steps
		cv::Mat predictState(cv::Mat controlInput);
		cv::Mat predictionCovariance();
		//update state
		cv::Mat kalmanGain(Mat jacobian, Mat obsNoise);
		cv::Mat getInnovation(Mat obs, Mat expectedObs);
		cv::Mat updateStep(Mat kalmanProduct);
		cv::Mat covarianceOperation();
		cv::Mat getCurrentState(){ return u_; }
		cv::Mat buildObsMat(int rows, CoordinateReal loc);
		void writeDataToFile(bool isPrediction);
	};
}

