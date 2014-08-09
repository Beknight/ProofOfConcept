#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "LogRecorder.h"
#include "Stereoscope.h"
class KalmanFilter
{
public:
	//setup
	KalmanFilter();
	~KalmanFilter();
	// kalman filter run
	void execute();
private:
	//the two required classes to help out 
	LogRecorder logRecorder_;
	Stereoscope stereoscope_;
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
	
};

