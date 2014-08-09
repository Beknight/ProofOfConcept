#pragma once
#include "LibraryHeaders.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include "CoordinateReal.h"
class FeatureExtraction
{
public:
	FeatureExtraction();
	void loadIntoLibrary(std::string subject);
	CoordinateReal detect(cv::Mat scene, bool debug);
	cv::Mat resultImage();
	~FeatureExtraction();
private: 
	//library templates 
	std::vector<cv::Mat> templates_;
	

};

