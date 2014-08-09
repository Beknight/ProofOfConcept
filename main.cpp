#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2\features2d\features2d.hpp"
#include "opencv2\nonfree\features2d.hpp"
#include "opencv2\calib3d\calib3d.hpp"
#include <iostream>
#include "Stereoscope.h"
#include "Camera.h"
using namespace cv;
using namespace std;

void main(int argc, char *argv[])
{
	//Camera one(2);
	//Camera two(0);
	Mat onePic;
	Mat twoPic;
	char pressedKey = ' ';
	float nndrRatio = 0.9;
	//===============opencv test========================
	//load in an image;
	cv::Mat sceneImage;
	cv::Mat subjectImage;
	sceneImage = cv::imread("../../../../ThesisImages/elf.jpg");
	subjectImage = cv::imread("../../../../ThesisImages/hands.jpg");
	Mat outImg = cv::imread("../../../../ThesisImages/elf.jpg");
	// keypoints
	std::vector<cv::KeyPoint> subjectKeyPoints;
	std::vector<cv::KeyPoint> sceneKeyPoints;
	cv::Mat subDescriptors, sceneDescriptors;
	//surf object that extracts keypoints 
	// larger hessian  means only pointy corners are found
	cv::SurfFeatureDetector surf(3000);
	surf.detect(subjectImage,subjectKeyPoints);
	surf.detect(sceneImage, sceneKeyPoints);
	//now to computer the keypoints
	cv::SurfDescriptorExtractor extractor;
	extractor.compute(subjectImage, subjectKeyPoints, subDescriptors);
	extractor.compute(sceneImage, sceneKeyPoints, sceneDescriptors);
	//flann based matcher
	FlannBasedMatcher matcher;
	vector< vector< DMatch >  > matches;
	matcher.knnMatch(subDescriptors, sceneDescriptors, matches, 2);
	//nearest neighbor mathc
	vector<DMatch> goodMatches;
	goodMatches.reserve(matches.size());
	for (size_t i = 0; i < matches.size(); ++i)
	{
		if (matches[i].size() < 2)
			continue;

		const DMatch &m1 = matches[i][0];
		const DMatch &m2 = matches[i][1];

		if (m1.distance <= nndrRatio * m2.distance)
			goodMatches.push_back(m1);
	}
	Mat img_matches;
	drawMatches(subjectImage, subjectKeyPoints, sceneImage, sceneKeyPoints,
		goodMatches, img_matches, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	//
	std::vector< Point2f >  obj;
	std::vector< Point2f >  scene;

	for (unsigned int i = 0; i < goodMatches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		obj.push_back(subjectKeyPoints[goodMatches[i].queryIdx].pt);
		scene.push_back(sceneKeyPoints[goodMatches[i].trainIdx].pt);
	}

	

	Mat H = findHomography(obj, scene, CV_RANSAC);


	//-- Get the corners from the image_1 ( the object to be "detected" )
	std::vector< Point2f > obj_corners(4);
	obj_corners[0] = cvPoint(0, 0);
	obj_corners[1] = cvPoint(subjectImage.cols, 0);
	obj_corners[2] = cvPoint(subjectImage.cols, subjectImage.rows);
	obj_corners[3] = cvPoint(0, subjectImage.rows);
	std::vector< Point2f > scene_corners(4);

	perspectiveTransform(obj_corners, scene_corners, H);
	cv::Scalar color(255, 0, 0);
	//-- Draw lines between the corners (the mapped object in the scene - image_2 )
	line(outImg, scene_corners[0], scene_corners[1], color, 2); //TOP line
	line(outImg, scene_corners[1], scene_corners[2], color, 2);
	line(outImg, scene_corners[2], scene_corners[3], color, 2);
	line(outImg, scene_corners[3], scene_corners[0], color, 2);
	imshow("hello", img_matches);
	cv::waitKey(0);
	//===============opencv test========================
	//while (1){
	//	onePic = one.grabFrame();
	//	twoPic = two.grabFrame();
	//	if (!onePic.empty() && !twoPic.empty()){
	//		imshow("one", onePic);
	//		pressedKey = cvWaitKey(3);
	//		imshow("two", twoPic);
	//		pressedKey = cvWaitKey(3);
	//	}
	//
	//	if (pressedKey == 'p'){
	//		imwrite("../../../../ThesisImages/onePic.jpg", onePic);
	//		imwrite("../../../../ThesisImages/twoPic.jpg", twoPic);
	//		Stereoscope stereo;
	//		double distance = stereo.getDistance();
	//		cout << "distance" <<distance << endl;
	//		cvWaitKey(10);
	//	}

	//}
	
	//initialise the required refernece images 

	// instantiate the kalman filter 

	//surf to find the feature 

	// while ! stop

		// predict state 

		//predict covariance

		//predict observation 

		// make observation 

		//kalman gain 

		// update state and covariance 

		// remember covariance and state
	
	//endwhile

}