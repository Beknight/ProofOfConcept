#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "LogRecorder.h"
#include "Stereoscope.h"
#include "Camera.h"
namespace Thesis{
	class KalmanFilter
	{
	public:
		//setup
		KalmanFilter();
		~KalmanFilter();
		void initialise(CoordinateReal initialPosition);
		cv::Mat expectedObservation(Camera camera);
		// kalman filter run
		void execute();
		void printCurrentState();
	private:
		// the angle constants that we need for 
		double angleConstX = 0;
		double angleConstY = 0;
		// observation 
		//the two required classes to help out 
		LogRecorder logRecorder_;

		// the four required kalman filter fields
		cv::Mat covariance_;
		cv::Mat q_;
		cv::Mat r_;
		cv::Mat x_;
		//prediction steps
		cv::Mat predictState(cv::Mat controlInput);
		cv::Mat predictionCovariance();
		//update state
		cv::Mat kalmanGain();
		cv::Mat getInnovation();
		cv::Mat updateStep();
		cv::Mat getCurrentState(){ return x_; }

	};
}

