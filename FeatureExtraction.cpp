#include "FeatureExtraction.h"


FeatureExtraction::FeatureExtraction(int minHess)
{
	surf_ = cv::SurfFeatureDetector(minHess);
}


FeatureExtraction::~FeatureExtraction()
{
}

std::vector<CoordinateReal> FeatureExtraction::detect(cv::Mat scene, bool debug)
{
	std::vector<CoordinateReal> objectLocationList;
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
		
		cv::Mat img_matches;
		cv::drawMatches(*templates_[i], libraryKeyPoints_[i], scene, sceneKeyPoints,
			goodMatches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
			std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		matchedImage_ = img_matches;
		std::vector< cv::Point2f >  obj;
		std::vector< cv::Point2f >  scenery;
		for (unsigned int matchCount = 0; matchCount < goodMatches.size(); matchCount++)
		{
			//-- Get the keypoints from the good matches
			obj.push_back(libraryKeyPoints_[i][goodMatches[matchCount].queryIdx].pt);
			scenery.push_back(sceneKeyPoints[goodMatches[matchCount].trainIdx].pt);
		}
		cv::Mat H = findHomography(obj, scenery, CV_RANSAC);
		cv::Scalar color(255, 0, 0);
		//-- Draw lines between the corners (the mapped object in the scene - image_2 )
		std::vector< cv::Point2f > obj_corners(4);
		obj_corners[0] = cvPoint(0, 0);
		obj_corners[1] = cvPoint(templates_[i]->cols, 0);
		obj_corners[2] = cvPoint(templates_[i]->cols, templates_[i]->rows);
		obj_corners[3] = cvPoint(0, templates_[i]->rows);
		std::vector< cv::Point2f > scene_corners(4);

		perspectiveTransform(obj_corners, scene_corners, H);
		//-- Draw lines between the corners (the mapped object in the scene - image_2 )
		line(matchedImage_, scene_corners[0] + cv::Point2f(templates_[i]->cols, 0), scene_corners[1] + cv::Point2f(templates_[i]->cols, 0), cv::Scalar(0, 255, 0), 2); //TOP line
		line(matchedImage_, scene_corners[1] + cv::Point2f(templates_[i]->cols, 0), scene_corners[2] + cv::Point2f(templates_[i]->cols, 0), cv::Scalar(0,0,255), 2);
		line(matchedImage_, scene_corners[2] + cv::Point2f(templates_[i]->cols, 0), scene_corners[3] + cv::Point2f(templates_[i]->cols, 0), color, 2);
		line(matchedImage_, scene_corners[3] + cv::Point2f(templates_[i]->cols, 0), scene_corners[0] + cv::Point2f(templates_[i]->cols, 0), color, 2);
		//average the lenght and the width of the box to get the approx centre
		double yAverage = 0.25*(scene_corners[0].y + scene_corners[1].y + scene_corners[2].y + scene_corners[3].y);
		double xAverage = 0.25*(scene_corners[0].x + scene_corners[1].x + scene_corners[2].x + scene_corners[3].x);
		int thickness = -1;
		int lineType = 8;
		cv::circle(matchedImage_, cv::Point2f(xAverage, yAverage),5,
			cv::Scalar(0, 0, 255),
			thickness,
			lineType);
		CoordinateReal objectLoc(xAverage, yAverage, 0);
		objectLocationList.push_back(objectLoc);
	}
	return objectLocationList;
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
