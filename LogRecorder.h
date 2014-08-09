#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
class LogRecorder
{
public:
	LogRecorder();
	~LogRecorder();

private:
	std::vector<cv::Point3d>  recordedHistory;
	
};

