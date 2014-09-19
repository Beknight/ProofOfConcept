#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2\features2d\features2d.hpp"
#include "opencv2\nonfree\features2d.hpp"
#include "opencv2\calib3d\calib3d.hpp"
#include <iostream>
#include "Stereoscope.h"
#include "FeatureExtraction.h"
#include "Camera.h"
#include "time.h"
#include "Util.h"
#include "KalmanFilter.h"
#include "FastTracking.h"
using namespace cv;
using namespace std;

void main(int argc, char *argv[])
{
	FastTracking fastTrack(20);
	VideoCapture capture;
				int thickness = -1;
				int lineType = 8;
	Mat frame1, frame2;
	while (1){
		//open the video
		capture.open("../../../../ThesisImages/bouncingBall.avi ");
		while (capture.get(CV_CAP_PROP_POS_FRAMES) < capture.get(CV_CAP_PROP_FRAME_COUNT) - 1){
			capture.read(frame1);
			waitKey(1);
			capture.read(frame2);

			waitKey(1);
			CoordinateReal loc = fastTrack.findObject(frame1, frame2);
			cv::circle(frame1, cv::Point2f(loc.x(), loc.y()), 5,
								cv::Scalar(0, 0, 255),
								thickness,
								lineType);
			imshow("fameOne", frame1);
		}

	}
	//KalmanFilter kalman;
	//// the two stereoscope images
	//Camera one(0,-125,0,0,0,90);
	//Camera two(2, 125,0,0,0,90);
	//bool found = false;
	//FeatureExtraction surf(5000);
	//Stereoscope stereo;
	//Util util;
	//bool running = true;
	//char curPressed = ' ';
	//surf.addImageToLib("backToTheFutureCover.jpg");
	//std::vector<cv::Point2f> leftRect(4);
	//cv::Rect leftRealRect;
	//cv::Rect rightRealRect;
	//std::vector<cv::Point2f> rightRect(4);
	//while (running){
	//	const clock_t beginTime = clock();
	//	curPressed = waitKey(10);

	//	cv::Mat frameLeft = one.grabFrame();
	//
	//	cv::Mat frameRight = two.grabFrame();

	//	frameRight = two.grabFrame();
	//	if (curPressed == ' '){
	//		//left frame =============================
	//		std::vector<CoordinateReal> coordLeft = surf.detect(frameLeft, true, found, leftRealRect);
	//		if (!coordLeft.empty()){
	//			int thickness = -1;
	//			int lineType = 8;
	//			cout << "x on camera: " << coordLeft[0].x() << " ; " << coordLeft[0].y();
	//			cv::circle(frameLeft, cv::Point2f(coordLeft[0].x(), coordLeft[0].y()), 5,
	//				cv::Scalar(0, 0, 255),
	//				thickness,
	//				lineType);
	//			leftRect = surf.getSceneCorners();
	//			line(frameLeft, leftRect[0], leftRect[1], cv::Scalar(0, 255, 0), 2); //TOP line
	//			line(frameLeft, leftRect[1], leftRect[2], cv::Scalar(0, 0, 255), 2);
	//			line(frameLeft, leftRect[2], leftRect[3], cv::Scalar(0, 255, 0), 2);
	//			line(frameLeft, leftRect[3], leftRect[0], cv::Scalar(0, 255, 0), 2);
	//			leftRealRect = util.getSizedRect(leftRect, one.reso_height, one.reso_width, 0.1);
	//		}
	//		//right frame ==================================
	//		std::vector<CoordinateReal> coordRight = surf.detect(frameRight, true, found, rightRealRect);
	//		if (!coordRight.empty()){
	//			int thickness = -1;
	//			int lineType = 8;
	//			cv::circle(frameRight, cv::Point2f(coordRight[0].x(), coordRight[0].y()), 5,
	//				cv::Scalar(0, 0, 255),
	//				thickness,
	//				lineType);
	//			rightRect = surf.getSceneCorners();
	//			line(frameRight, rightRect[0], rightRect[1], cv::Scalar(0, 255, 0), 2); //TOP line
	//			line(frameRight, rightRect[1], rightRect[2], cv::Scalar(0, 0, 255), 2);
	//			line(frameRight, rightRect[2], rightRect[3], cv::Scalar(0, 255, 0), 2);
	//			line(frameRight, rightRect[3], rightRect[0], cv::Scalar(0, 255, 0), 2);
	//			rightRealRect = util.getSizedRect(rightRect, one.reso_height, one.reso_width, 0.1);
	//		}
	//		////char curPressed = cvWaitKey(100);
	//		//if (curPressed == pressedKey){
	//		//	one.writeLatestImageToFile("../../../../ThesisImages/povTest.jpg");
	//		//	running = false;
	//		//}
	//		//else if (curPressed == ' '){
	//		//	running = false;
	//		//}
	//		CoordinateReal real = stereo.getLocation(coordLeft[0], coordRight[0]);
	//		cout << "z: " << real.z() << " x: " << real.x() << " y: " << real.y();
	//		kalman.initialise(real);
	//		kalman.printCurrentState();
	//		cv::imshow("left", frameLeft);
	//		cv::imshow("right", frameRight);
	//		cout <<"time in seconds" <<float(clock() - beginTime) / CLOCKS_PER_SEC << endl;
	//		found = true;
	//		kalman.initialise(real);
	//		kalman.expectedObservation(one);
	//		char secondKey = cv::waitKey(0);
	//		if (secondKey == 'r'){
	//			found = false;
	//		}
	//	}
	//	cv::imshow("left", frameLeft);
	//	cv::imshow("right", frameRight);
	//}

}
