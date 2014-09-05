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
using namespace cv;
using namespace std;

void main(int argc, char *argv[])
{
	Camera one(0);
	Camera two(2);
	bool found = false;
	FeatureExtraction surf(7000);
	Stereoscope stereo;
	Util util;
	bool running = true;
	char curPressed = ' ';
	surf.addImageToLib("backToTheFutureCover.jpg");
	std::vector<cv::Point2f> leftRect(4);
	cv::Rect leftRealRect;
	cv::Rect rightRealRect;
	std::vector<cv::Point2f> rightRect(4);
	while (running){
		const clock_t beginTime = clock();
		curPressed = waitKey(10);
		//////left frame =============================
		cv::Mat frameLeft = one.grabFrame();
	
		cv::Mat frameRight = two.grabFrame();

		frameRight = two.grabFrame();
		if (curPressed == ' '){
			//char pressedKey = 'p';
			////
			
			std::vector<CoordinateReal> coordLeft = surf.detect(frameLeft, true, found, leftRealRect);
			if (!coordLeft.empty()){
				int thickness = -1;
				int lineType = 8;
				cv::circle(frameLeft, cv::Point2f(coordLeft[0].x(), coordLeft[0].y()), 5,
					cv::Scalar(0, 0, 255),
					thickness,
					lineType);
				leftRect = surf.getSceneCorners();
				line(frameLeft, leftRect[0], leftRect[1] , cv::Scalar(0, 255, 0), 2); //TOP line
				line(frameLeft, leftRect[1], leftRect[2] , cv::Scalar(0, 0, 255), 2);
				line(frameLeft, leftRect[2], leftRect[3], cv::Scalar(0, 255, 0), 2);
				line(frameLeft, leftRect[3], leftRect[0], cv::Scalar(0, 255, 0), 2);
				if (!found){
					leftRealRect = util.getSizedRect(leftRect, one.reso_height, one.reso_width, 0.1);
				}
			}

			////right frame ==================================
			std::vector<CoordinateReal> coordRight = surf.detect(frameRight, true, found, rightRealRect);
			if (!coordRight.empty()){
				int thickness = -1;
				int lineType = 8;
				cv::circle(frameRight, cv::Point2f(coordRight[0].x(), coordRight[0].y()), 5,
					cv::Scalar(0, 0, 255),
					thickness,
					lineType);

				rightRect = surf.getSceneCorners();
				line(frameRight, rightRect[0], rightRect[1], cv::Scalar(0, 255, 0), 2); //TOP line
				line(frameRight, rightRect[1], rightRect[2], cv::Scalar(0, 0, 255), 2);
				line(frameRight, rightRect[2], rightRect[3], cv::Scalar(0, 255, 0), 2);
				line(frameRight, rightRect[3], rightRect[0], cv::Scalar(0, 255, 0), 2);
				if (!found){
					rightRealRect = util.getSizedRect(rightRect, one.reso_height, one.reso_width, 0.1);
				}
			}
			////char curPressed = cvWaitKey(100);
			//if (curPressed == pressedKey){
			//	one.writeLatestImageToFile("../../../../ThesisImages/povTest.jpg");
			//	running = false;
			//}
			//else if (curPressed == ' '){
			//	running = false;
			//}
			CoordinateReal real = stereo.getLocation(coordLeft[0], coordRight[0]);
			cv::imshow("left", frameLeft);
			cv::imshow("right", frameRight);
			cout <<"time in seconds" <<float(clock() - beginTime) / CLOCKS_PER_SEC << endl;
			found = true;
			cv::waitKey(0);
		}
		cv::imshow("left", frameLeft);
		cv::imshow("right", frameRight);
	}

}
