#pragma once
#include "LibraryHeaders.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <memory>
#include "CoordinateReal.h"
class FeatureExtraction
{
public:
	FeatureExtraction(int minHess);
	void addImageToLib(std::string subject);
	CoordinateReal detect(cv::Mat scene, bool debug);

	cv::Mat resultImage() { return matchedImage_; }
	~FeatureExtraction();
private: 
	//library templates 
	int libraryCount_ = 0;
	float nndrRatio_ = 0.7;
	std::vector<std::shared_ptr<cv::Mat>> templates_;
	//2d a vector of a vector of keypoints
	std::vector<std::vector<cv::KeyPoint>> libraryKeyPoints_;
	std::vector<std::shared_ptr<cv::Mat>> libraryDescriptors_;
	//Surf required objects
	cv::SurfFeatureDetector surf_;
	cv::SurfFeatureDetector extractor_;
	cv::FlannBasedMatcher matcher_;
	//the image with the subject located
	cv::Mat matchedImage_;



};

