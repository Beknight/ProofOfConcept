#pragma once
#include "LibraryHeaders.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <memory>
#include "CoordinateReal.h"
#include "opencv2\nonfree\gpu.hpp"
class FeatureExtraction
{
public:
	FeatureExtraction(int minHess);
	void addImageToLib(std::string subject);
	std::vector<CoordinateReal> detect(cv::Mat scene, bool debug, bool found, cv::Rect subMat);
	CoordinateReal grabObservation(cv::Mat scene, bool debug, bool found, cv::Rect subMat);
	std::vector<cv::Point2f> getSceneCorners() { return sceneCorners_; }
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
	std::vector<cv::Point2f> sceneCorners_;
	std::vector<cv::Point2f> getDelta(std::vector<cv::Point2f>, int deltaX, int deltaY);
	cv::Mat applyMask(float percentage, cv::Mat src);
};

