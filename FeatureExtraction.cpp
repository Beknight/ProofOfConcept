#include "FeatureExtraction.h"


FeatureExtraction::FeatureExtraction(int minHess)
{
	surf_ = cv::SurfFeatureDetector(minHess);
}


FeatureExtraction::~FeatureExtraction()
{
}

CoordinateReal FeatureExtraction::detect(cv::Mat scene, bool debug)
{
	CoordinateReal objectLocation(0,0,0);
	////get the detectors of the scene
	std::vector<cv::KeyPoint> sceneKeyPoints;
	cv::Mat sceneDescriptors;
	surf_.detect(scene, sceneKeyPoints);
	//get the descriptors
	extractor_.compute(scene, sceneKeyPoints, sceneDescriptors);
	////for all the template images, search the scene
	for (int i = 0; i < libraryCount_; i++){
		std::vector<std::vector<cv::DMatch>> matches;
		//	//grab the set of descriptors
		matcher_.knnMatch(*libraryDescriptors_[i], sceneDescriptors, matches, 2);
		std::vector<cv::DMatch> goodMatches;
		goodMatches.reserve(matches.size());
		//	// using nearest neighbor ratio, find the matches that are most likely
		for (size_t x = 0; x < matches.size(); x++)
		{
			if (matches[x].size() < 2)
				continue;

			const cv::DMatch &m1 = matches[x][0];
			const cv::DMatch &m2 = matches[x][1];

			if (m1.distance <= nndrRatio_ * m2.distance)
				goodMatches.push_back(m1);
		}
		//std::vector< cv::Point2f >  obj;
		//std::vector< cv::Point2f >  scene;
		cv::Mat img_matches;
		cv::drawMatches(*templates_[i], libraryKeyPoints_[i], scene, sceneKeyPoints,
			goodMatches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
			std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		matchedImage_ = img_matches;
		//		//for (unsigned int i = 0; i < goodMatches.size(); i++)
		//		//{
		//		//	//-- Get the keypoints from the good matches
		//		//	obj.push_back(libraryKeyPoints_[i][goodMatches[i].queryIdx].pt);
		//		//	scene.push_back(sceneKeyPoints[goodMatches[i].trainIdx].pt);
		//		//}
	}
	return objectLocation;
}

void FeatureExtraction::addImageToLib(std::string subject)
{
	//create a new vector of keypoints
	std::vector<cv::KeyPoint> subjectKeyPoints;
	//create a vector of descriptors
	std::shared_ptr<cv::Mat> descriptor(new cv::Mat());
	std::string pwd = "../../../../ThesisImages/";
	pwd += subject;
	//create a raw pointer for the reference image
	//wrap it in shared pointer
	std::shared_ptr<cv::Mat> img_ptr(new cv::Mat(cv::imread(pwd)));
	//push onto the vector
	templates_.push_back(img_ptr);
	surf_.detect(*img_ptr, subjectKeyPoints);
	//add the vector into the library 
	libraryKeyPoints_.push_back(subjectKeyPoints);
	//gather the descriptors
	extractor_.compute(*img_ptr, subjectKeyPoints, *descriptor);
	libraryDescriptors_.push_back(descriptor);
	libraryCount_++;
}

