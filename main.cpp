#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2\features2d\features2d.hpp"
#include "opencv2\nonfree\features2d.hpp"
#include "opencv2\calib3d\calib3d.hpp"
#include <iostream>
#include "Stereoscope.h"
#include "FeatureExtraction.h"
#include "Camera.h"
using namespace cv;
using namespace std;

void main(int argc, char *argv[])
{
	Camera one(1);
	//Camera two(2);
	FeatureExtraction surf(3000);
	/*surf.addImageToLib("hands.jpg");
	cv::Mat image = cv::imread("../../../../ThesisImages/elf.jpg");
	surf.detect(image, true);
	cv::imshow("winning", surf.resultImage());
	waitKey(0);*/
	bool running = true;
	while (running){
		cv::Mat frame = one.grabFrame();
		char pressedKey = ' ';
		imshow("hello", frame);
		char curPressed = cvWaitKey(3);
		if (curPressed == pressedKey){
			one.writeLatestImageToFile("povTest");
			running = false;
		}
	}

	//Mat onePic;
	//Mat twoPic;
	//char pressedKey = ' ';
	//float nndrRatio = 0.9;
	////===============opencv test========================

	//std::vector< Point2f >  obj;
	//std::vector< Point2f >  scene;

	//for (unsigned int i = 0; i < goodMatches.size(); i++)
	//{
	//	//-- Get the keypoints from the good matches
	//	obj.push_back(subjectKeyPoints[goodMatches[i].queryIdx].pt);
	//	scene.push_back(sceneKeyPoints[goodMatches[i].trainIdx].pt);
	//}

	//Mat H = findHomography(obj, scene, CV_RANSAC);


	////-- Get the corners from the image_1 ( the object to be "detected" )
	//std::vector< Point2f > obj_corners(4);
	//obj_corners[0] = cvPoint(0, 0);
	//obj_corners[1] = cvPoint(subjectImage.cols, 0);
	//obj_corners[2] = cvPoint(subjectImage.cols, subjectImage.rows);
	//obj_corners[3] = cvPoint(0, subjectImage.rows);
	//std::vector< Point2f > scene_corners(4);

	//perspectiveTransform(obj_corners, scene_corners, H);
	//cv::Scalar color(255, 0, 0);
	////-- Draw lines between the corners (the mapped object in the scene - image_2 )
	//line(outImg, scene_corners[0], scene_corners[1], color, 2); //TOP line
	//line(outImg, scene_corners[1], scene_corners[2], color, 2);
	//line(outImg, scene_corners[2], scene_corners[3], color, 2);
	//line(outImg, scene_corners[3], scene_corners[0], color, 2);
	//imshow("hello", img_matches);
	//cv::waitKey(0);
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