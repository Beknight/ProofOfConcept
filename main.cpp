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
bool pause = false;
bool surfing = false;
bool leftDebug = false;
bool rightDebug = false;
bool record = false;
void main(int argc, char *argv[])
{
	Mat emptyFrame = Mat::zeros(Camera::reso_height, Camera::reso_width, CV_8UC3);
	Thesis::FastTracking fastTrack(16);
	Thesis::KalmanFilter kalman;
	// the two stereoscope images
	Camera one(0,-125,0,0,0,90);
	Camera two(2, 125,0,0,0,90);
	VideoWriter writeOne ;
	VideoWriter writeTwo;
	VideoCapture capOne;
	VideoCapture capTwo;
	//open the recorders
	//writeOne.open("../../../../ThesisImages/leftFast.avi", 0, 12, cv::Size(864, 480), true);
	//writeTwo.open("../../../../ThesisImages/rightFast.avi", 0, 12, cv::Size(864, 480), true);
	FeatureExtraction surf(5000);
	Stereoscope stereo;
	Util util;
	bool once = false;
	bool foundInBoth = false;
	std::vector<cv::Point2f> leftRect(4);
	cv::Rect leftRealRect;
	cv::Rect rightRealRect;
	std::vector<cv::Point2f> rightRect(4);
	cv::Mat frameLeft;
	cv::Mat frameRight;
	cv::Mat prevFrameLeft;
	cv::Mat prevFrameRight;
	string left = "../../../../ThesisImages/left.avi";
	string right = "../../../../ThesisImages/right.avi";
	// check if you going to run simulation or not
	cout << " run simulation: 's' or normal 'n'" << endl;
	imshow("main", emptyFrame);
	char command = waitKey(0);
	commands(command);
	if (simulation){
		capOne.open(left);
		capTwo.open(right);
		assert(capOne.isOpened() && capTwo.isOpened());
	}
	else{
		surf.addImageToLib("backToTheFutureCover.jpg");
	}
	CoordinateReal leftLoc;
	CoordinateReal rightLoc;
	while (running){
		const clock_t beginTime = clock();
		command = waitKey(3);
		commands(command);
		int thickness = -1;
		int lineType = 8;
		if (!simulation){
			frameLeft = one.grabFrame();
			frameRight = two.grabFrame();
		}
		else{
			// if last frame, release then reopen
			if (capOne.get(CV_CAP_PROP_POS_FRAMES) == (capOne.get(CV_CAP_PROP_FRAME_COUNT) - 1)){
				capOne.release();
				capTwo.release();
				capOne.open(left);
				capTwo.open(right);
			}
			// means it is simulation: i.e frames come from a video
			capOne >> frameLeft;
			capTwo >> frameRight;
		}
		//writeOne.write(frameLeft);
		//writeTwo.write(frameRight);
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
				leftLoc = coordLeft[0];
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
				rightLoc = coordRight[0];
			}
			////char curPressed = cvWaitKey(100);
			//if (curPressed == pressedKey){
			//	one.writeLatestImageToFile("../../../../ThesisImages/povTest.jpg");
			//	running = false;
			//}
			//else if (curPressed == ' '){
			//	running = false;
			//}
			found = true;
		}
		else{
			if (once){
				leftLoc = fastTrack.findObject(frameLeft, prevFrameLeft, leftDebug);
				rightLoc = fastTrack.findObject(frameRight, prevFrameRight, rightDebug);
			}
			frameLeft.copyTo(prevFrameLeft);
			frameRight.copyTo(prevFrameRight);
			once = true;
			cv::circle(frameLeft, cv::Point2f(leftLoc.x(), leftLoc.y()), 5,
				cv::Scalar(0, 0, 255),
				thickness,
				lineType);
			cv::circle(frameRight, cv::Point2f(rightLoc.x(), rightLoc.y()), 5,
				cv::Scalar(0, 0, 255),
				thickness,
				lineType);
		}
		cv::imshow("left", frameLeft);
		cv::imshow("right", frameRight);
		foundInBoth = Util::isInBothFrames(leftLoc, rightLoc);
		
		if (foundInBoth){
			cout << "found in both" << endl;
			CoordinateReal real = stereo.getLocation(leftLoc, rightLoc);
			cout << "z: " << real.z() << " x: " << real.x() << " y: " << real.y();
			kalman.initialise(real);
			cv::imshow("left", frameLeft);
			cv::imshow("right", frameRight);
			cout << "time in seconds" << float(clock() - beginTime) / CLOCKS_PER_SEC << endl;
			if (surfing){
				waitKey(0);
				surfing = false;
			}
			kalman.initialise(real);
			kalman.expectedObservation(one);
		}
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
	case 'd':
		//debug
		break;
	case ' ':
	// surf scanning
		surfing = true;
		break;
	case 'r':
	// reset
		found = false;
		break;
	case 's':
	//simulation mode
		simulation = true;
		break;
	case 'p':
		waitKey(0);
		break;
	case 'n':
	//normal mode
		simulation = false;
		break;
	case 'k':
		rightDebug = true;
		leftDebug = false;
		break;
	case 'l':
		leftDebug = true;
		rightDebug = false;
		break;
	}
}