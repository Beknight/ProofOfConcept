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
bool hsv = false;
void main(int argc, char *argv[])
{
	Mat emptyFrame = Mat::zeros(Camera::reso_height, Camera::reso_width, CV_8UC3);
	Thesis::FastTracking fastTrack(16);
	Thesis::KalmanFilter kalman;
	kalman.initialise(CoordinateReal(0, 0, 0));
	kalman.openFile();
	// the two stereoscope images
	Camera one(0,-125,0,0,0,90);
	Camera two(2, 125,0,0,0,90);
	VideoWriter writeOne ;
	VideoWriter writeTwo;
	VideoCapture capOne;
	VideoCapture capTwo;
	double framesPerSecond = 1 / 10.0;
	//open the recorders
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

	// check if you going to run simulation or not or record
	cout << " run simulation: 's' or normal 'n' or 'o'" << endl;
	imshow("main", emptyFrame);
	char command = waitKey(0);
	string left = "../../../../ThesisImages/left.avi";
	string right = "../../../../ThesisImages/right.avi";

	commands(command);
	//==========hsv values=======================
	cv::Mat hsvFrame;
	cv::Mat threshold;
	int iLowH = 14;
	int iHighH = 179;

	int iLowS = 131;
	int iHighS = 255;

	int iLowV = 57;
	int iHighV = 255;
	
	//=================================

	if (record){
		writeOne.open("../../../../ThesisImages/leftFast.avi", 0, 15, cv::Size(864, 480), true);
		writeTwo.open("../../../../ThesisImages/rightFast.avi", 0, 15, cv::Size(864, 480), true);
	}else if (simulation){
		capOne.open(left);
		capTwo.open(right);
		assert(capOne.isOpened() && capTwo.isOpened());
	}
	else if (hsv){
		//Create trackbars in "Control" window
		cvCreateTrackbar("LowH", "main", &iLowH, 179); //Hue (0 - 179)
		cvCreateTrackbar("HighH", "main", &iHighH, 179);

		cvCreateTrackbar("LowS", "main", &iLowS, 255); //Saturation (0 - 255)
		cvCreateTrackbar("HighS", "main", &iHighS, 255);

		cvCreateTrackbar("LowV", "main", &iLowV, 255); //Value (0 - 255)
		cvCreateTrackbar("HighV", "main", &iHighV, 255);
	}
	else{
		surf.addImageToLib("backToTheFutureCover.jpg");
	}
	CoordinateReal leftLoc;
	CoordinateReal rightLoc;
	while (running){
		const clock_t beginTime = clock();
		command = waitKey(1);
		commands(command);
		kalman.predictState();
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
			if (hsv){
				//convert the frame into hsv
				cvtColor(frameRight, hsvFrame, COLOR_BGR2HSV);
				inRange(hsvFrame, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), threshold);
				blur(threshold, threshold, cv::Size(20, 20));
				cv::threshold(threshold,threshold,60,255,THRESH_BINARY);
				////morphological opening (remove small objects from the foreground)
				//erode(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				//dilate(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				//dilate(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				//erode(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				imshow("imageTwo", hsvFrame);
				imshow("hsv", threshold);
			}
		}
		if (record){
			writeOne.write(frameLeft);
			writeTwo.write(frameRight);
		}
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
		else if(!record){
			if (once){
				CoordinateReal leftCameraLoc = kalman.expectedLocObs(one);
				CoordinateReal rightCameraLoc = kalman.expectedLocObs(two);
				leftLoc = fastTrack.findObject(frameLeft, prevFrameLeft, leftCameraLoc,leftDebug);
				rightLoc = fastTrack.findObject(frameRight, prevFrameRight, rightCameraLoc ,rightDebug);
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
		
		foundInBoth = Util::isInBothFrames(leftLoc, rightLoc);
	
		if (foundInBoth){
			CoordinateReal real = stereo.getLocation(leftLoc, rightLoc);
			cout << "z: " << real.z() << " x: " << real.x() << " y: " << real.y();
			//cout << "time in seconds" << float(clock() - beginTime) / CLOCKS_PER_SEC << endl;
			if (surfing){
				waitKey(0);
				surfing = false;
			}
			if (!found){
				kalman.initialise(real);
			}
			foundInBoth = false;
			found = true;
		}
		cv::imshow("left", frameLeft);
		cv::imshow("right", frameRight);
		double timeElapsed = double(clock() - beginTime) / CLOCKS_PER_SEC;
		double waitDelta = framesPerSecond - timeElapsed;
		
		if (waitDelta > 0){
			waitKey(waitDelta*1000);
		}
		cout << "fps:" << 1/(double(clock() - beginTime) / CLOCKS_PER_SEC) << endl;
	}
	kalman.closeFile();
	return;
}

void commands(char c){
	switch (c){
	case 'h':
		hsv = true;
		simulation = true;

		break;
	case 'o':
		record = true;
		simulation = false;
		break;
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
		record = false;
		break;
	case 'p':
		waitKey(0);
		break;
	case 'n':
	//normal mode
		simulation = false;
		record = false;
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