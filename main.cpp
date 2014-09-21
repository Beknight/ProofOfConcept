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
// functions
void commands(char command);
// flags
bool simulation = false;
bool running = true;
bool found = false;
bool screenShot = false;

void main(int argc, char *argv[])
{
	Mat emptyFrame = Mat::zeros(Camera::reso_height, Camera::reso_width, CV_8UC3);
	Thesis::FastTracking fastTrack(19);
	Thesis::KalmanFilter kalman;
	// the two stereoscope images
	Camera one(0,-125,0,0,0,90);
	Camera two(2, 125,0,0,0,90);
	VideoWriter writeOne ;
	VideoWriter writeTwo;
	VideoCapture capOne;
	VideoCapture capTwo;
	//open the recorders
	//capOne.open("../../../../ThesisImages/left.avi", 0, 10, cv::Size(864, 480), true);
	//capTwo.open("../../../../ThesisImages/right.avi", 0, 10, cv::Size(864, 480), true);
	FeatureExtraction surf(5000);
	Stereoscope stereo;
	Util util;
	bool once = false;
	surf.addImageToLib("backToTheFutureCover.jpg");
	std::vector<cv::Point2f> leftRect(4);
	cv::Rect leftRealRect;
	cv::Rect rightRealRect;
	std::vector<cv::Point2f> rightRect(4);
	cv::Mat frameLeft;
	cv::Mat frameRight;
	cv::Mat prevFrame;
	// check if you going to run simulation or not
	cout << " run simulation: 's' or normal 'n'" << endl;
	imshow("main", emptyFrame);
	char command = waitKey(0);
	commands(command);
	if (simulation){
		string left = "../../../../ThesisImages/left.avi";
		string right = "../../../../ThesisImages/right.avi";
		capOne.open(left);
		capTwo.open(right);
		assert(capOne.isOpened() && capTwo.isOpened());
	}
	while (running){
		const clock_t beginTime = clock();
		command = waitKey(3);
		int thickness = -1;
		int lineType = 8;
		if (!simulation){
			frameLeft = one.grabFrame();
			frameRight = two.grabFrame();
		}
		else{
			// means it is simulation: i.e frames come from a video
			capOne >> frameLeft;
			capTwo >> frameRight;
		}
		//capOne.write(frameLeft);
		//capTwo.write(frameRight);
		if (command == ' '){
			//left frame =============================
			std::vector<CoordinateReal> coordLeft = surf.detect(frameLeft, true, found, leftRealRect);
			if (!coordLeft.empty()){
				int thickness = -1;
				int lineType = 8;
				cv::circle(frameLeft, cv::Point2f(coordLeft[0].x(), coordLeft[0].y()), 5,
					cv::Scalar(0, 0, 255),
					thickness,
					lineType);
				leftRect = surf.getSceneCorners();
				line(frameLeft, leftRect[0], leftRect[1], cv::Scalar(0, 255, 0), 2); //TOP line
				line(frameLeft, leftRect[1], leftRect[2], cv::Scalar(0, 0, 255), 2);
				line(frameLeft, leftRect[2], leftRect[3], cv::Scalar(0, 255, 0), 2);
				line(frameLeft, leftRect[3], leftRect[0], cv::Scalar(0, 255, 0), 2);
				leftRealRect = util.getSizedRect(leftRect, one.reso_height, one.reso_width, 0.1);
			}
			//right frame ==================================
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
				rightRealRect = util.getSizedRect(rightRect, one.reso_height, one.reso_width, 0.1);
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
			cout << "z: " << real.z() << " x: " << real.x() << " y: " << real.y();
			kalman.initialise(real);
			kalman.printCurrentState();
			cv::imshow("left", frameLeft);
			cv::imshow("right", frameRight);
			cout <<"time in seconds" <<float(clock() - beginTime) / CLOCKS_PER_SEC << endl;
			found = true;
			kalman.initialise(real);
			kalman.expectedObservation(one);
			char secondKey = cv::waitKey(0);
			if (secondKey == 'r'){
				found = false;
			}
		}
		CoordinateReal loc;
		CoordinateReal locTwo;
		if (once){
			loc = fastTrack.findObject(frameLeft, prevFrame);
		}
		frameLeft.copyTo(prevFrame);
		once = true;
	
		cv::circle(frameLeft, cv::Point2f(loc.x(), loc.y()), 5,
			cv::Scalar(0, 0, 255),
			thickness,
			lineType);
		cv::imshow("left", frameLeft);
		cv::imshow("right", frameRight);
		//waitKey(10);
		commands(command);
	}
	return;
}

void commands(char c){
	switch (c){
	case 'x':
	// exit 
		running = false;
		cout << " exit " << endl;
		break;
	case 'f':
	// fast tracking
	
		break;
	case ' ':
	// surf scanning
		break;
	case 'r':
	// reset
		break;
	case 's':
	//simulation mode
		simulation = true;
		break;
	case 'n':
	//normal mode
		simulation = false;
		break;
	}
}