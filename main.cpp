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
#include "Stats.h"
#include <fstream>
using namespace cv;
using namespace std;
// functions
void commands(char command);
void camCount(char command);
// flags
bool simulation = false;
bool running = true;
bool found = false;
bool screenShot = false;
bool pause = false;
bool surfing = false;
bool leftDebug = false;
bool rightDebug = false;
bool threeDebug = false;
bool record = false;
bool hsv = false;
bool multiCams = false;
bool cross = false;
void main(int argc, char *argv[])
{
	Mat emptyFrame = Mat::zeros(Camera::reso_height, Camera::reso_width, CV_8UC3);
	Thesis::FastTracking fastTrack(20); //used to be 50, why? i dno
	Thesis::KalmanFilter kalman;
	kalman.initialise(CoordinateReal(0, 0, 0));
	kalman.openFile();
	// the two stereoscope images
	Camera one(0,-125,0,0,0,90);
	Camera two(2, 125,0,0,0,90);
	Camera three;
	// list of cameras and cameraLocs
	std::vector<Camera> cameraList;
	std::vector<CoordinateReal> locList;
	VideoWriter writeOne ;
	VideoWriter writeTwo;
	VideoWriter writeThree;
	VideoCapture capOne;
	VideoCapture capTwo;
	VideoCapture capThree;
	Thesis::Stats stat;
	cv::Point2d horizontalOne(0,Camera::reso_height/2);
	cv::Point2d horizontalTwo(Camera::reso_width, Camera::reso_height/2);
	cv::Point2d verticalOne(Camera::reso_width / 2, 0);
	cv::Point2d verticalTwo(Camera::reso_width / 2, Camera::reso_height);
	ofstream framesFile_;
	framesFile_.open("../../../../ThesisImages/fps_ABSDIFF.txt");
	double framesPerSecond = 1 / 10.0;
	//open the recorders
	FeatureExtraction surf(5000);
	Stereoscope stereo;
	Util util;
	bool once = false;
	bool foundInBoth = false;
	bool foundInMono = false;
	std::vector<cv::Point2f> leftRect(4);
	cv::Rect leftRealRect;
	cv::Rect rightRealRect;
	std::vector<cv::Point2f> rightRect(4);
	cv::Mat frameLeft;
	cv::Mat frameRight;
	cv::Mat frameThree;
	cv::Mat prevFrameLeft;
	cv::Mat prevFrameRight;
	cv::Mat prevFrameThree;

	// check if you going to run simulation or not or record
	cout << " run simulation: 's' or normal: 'n' or record 'o' or threeCameras 'c' " << endl;
	imshow("main", emptyFrame);
	char command = waitKey(0);

	string left = "../../../../ThesisImages/leftTen.avi";
	string right = "../../../../ThesisImages/rightTen.avi";
	string mid = "../../../../ThesisImages/midTen.avi";
	commands(command);
	emptyFrame = Mat::ones(10, 10, CV_64F);
	imshow("main", emptyFrame);
	command = waitKey(0);
	camCount(command);
	// checkt the cam count 
	if (multiCams){
		//load in all the cameras
		three = Camera(3, 0, 0, 0, 0, 90);//Camera(3, 200, -60, 480, 7,111);
	}
	//==========hsv values=======================
	cv::Mat hsvFrame;
	cv::Mat threshold;
	int iLowH = 155;
	int iHighH = 179;

	int iLowS = 75;
	int iHighS = 255;

	int iLowV = 0;
	int iHighV = 255;
	
	//=================================
	double elapsedTime = 0;
	double waitDelta = 0;	
	if (record){
		writeOne.open("../../../../ThesisImages/leftTen.avi", 0, 10, cv::Size(864, 480), true);
		writeTwo.open("../../../../ThesisImages/rightTen.avi", 0, 10, cv::Size(864, 480), true);
		writeThree.open("../../../../ThesisImages/midTen.avi", 0, 10, cv::Size(864, 480), true);
	}else if (simulation){
		capOne.open(left);
		capTwo.open(right);
		capThree.open(mid);
		assert(capOne.isOpened() && capTwo.isOpened());
	}
	 if (hsv){
		//Create trackbars in "Control" window
		cvCreateTrackbar("LowH", "main", &iLowH, 179); //Hue (0 - 179)
		cvCreateTrackbar("HighH", "main", &iHighH, 179);

		cvCreateTrackbar("LowS", "main", &iLowS, 255); //Saturation (0 - 255)
		cvCreateTrackbar("HighS", "main", &iHighS, 255);

		cvCreateTrackbar("LowV", "main", &iLowV, 255); //Value (0 - 255)
		cvCreateTrackbar("HighV", "main", &iHighV, 255);
	}
	if(!simulation){
		cout << " adding" << endl;
		surf.addImageToLib("backToTheFutureCover.jpg");
	}
	CoordinateReal leftLoc;
	CoordinateReal rightLoc;
	CoordinateReal threeLoc;
	while (running){
		clock_t beginTime = clock();
		commands(command);
		if (found){
			kalman.predictState();
			kalman.printCurrentState();
		}
		int thickness = -1;
		int lineType = 8;
		//normal running
		if (!simulation){
			frameLeft = one.grabFrame();
			frameRight = two.grabFrame();
			if (multiCams){
				frameThree = three.grabFrame();
			}
		}
		else{
			 //if last frame, release then reopen
			if (capOne.get(CV_CAP_PROP_POS_FRAMES) == (capOne.get(CV_CAP_PROP_FRAME_COUNT) - 1)){
				capOne.release();
				capTwo.release();
				capOne.open(left);
				capTwo.open(right);
				if (multiCams){
					capThree.release();
					capThree.open(mid);
				}
			}
			// means it is simulation: i.e frames come from a video
			capOne >> frameLeft;
			capTwo >> frameRight;
			if (multiCams){
				capThree >> frameThree;
			}
		}
		if (hsv){
			//convert the frame into hsv
			cvtColor(frameLeft, hsvFrame, COLOR_BGR2HSV);
			inRange(hsvFrame, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), threshold);
			blur(threshold, threshold, cv::Size(20, 20));
			cv::threshold(threshold, threshold, 50, 255, THRESH_BINARY);
			//imshow("imageTwo", hsvFrame);
			imshow("hsv", threshold);
		}
	
		if (record){
			writeOne.write(frameLeft);
			writeTwo.write(frameRight);
			if (multiCams){
				writeThree.write(frameThree);
			}
		}
		if (command == ' '){
			//left frame =============================
			cout << "pressedSpace " << endl;
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
			if (multiCams){
				std::vector<CoordinateReal> coordThrees = surf.detect(frameThree, true, false, leftRealRect);
				CoordinateReal coordThree = coordThrees[0];
				rightRect = surf.getSceneCorners();
				line(frameThree, rightRect[0], rightRect[1], cv::Scalar(0, 255, 0), 2); //TOP line
				line(frameThree, rightRect[1], rightRect[2], cv::Scalar(0, 0, 255), 2);
				line(frameThree, rightRect[2], rightRect[3], cv::Scalar(0, 255, 0), 2);
				line(frameThree, rightRect[3], rightRect[0], cv::Scalar(0, 255, 0), 2);
				cout << " foundIN x: " << coordThree.x() << "found in y: " << coordThree.y() << endl;
			}
			found = true;
		}
		else if(!record){
			cout << " fastTracking " << endl;
			if (once){
				CoordinateReal leftCameraLoc(0, 0, 0);
				CoordinateReal rightCameraLoc(0,0,0);
				if (found) {
					leftCameraLoc = kalman.expectedLocObs(one);
					rightCameraLoc = kalman.expectedLocObs(two);
				}
				leftLoc = fastTrack.findObject(frameLeft, prevFrameLeft, leftCameraLoc,leftDebug);
				rightLoc = fastTrack.findObject(frameRight, prevFrameRight, rightCameraLoc ,rightDebug);
				// go through the list of locations 
				if (multiCams){
					CoordinateReal miscCameraLoc(0, 0, 0);
					if (found){
						miscCameraLoc = kalman.expectedLocObs(three);
					}
					threeLoc = fastTrack.findObject(frameThree, prevFrameThree, miscCameraLoc, threeDebug);
				}
			}
			frameLeft.copyTo(prevFrameLeft);
			frameRight.copyTo(prevFrameRight);
			if (multiCams){
				frameThree.copyTo(prevFrameThree);
			}
			once = true;
			cv::circle(frameLeft, cv::Point2f(leftLoc.x(), leftLoc.y()), 5,
				cv::Scalar(0, 0, 255),
				thickness,
				lineType);
			cv::circle(frameRight, cv::Point2f(rightLoc.x(), rightLoc.y()), 5,
				cv::Scalar(0, 0, 255),
				thickness,
				lineType);
			cv::circle(frameThree, cv::Point2f(threeLoc.x(), threeLoc.y()), 5,
				cv::Scalar(0, 0, 255),
				thickness,
				lineType);
		}
		if (multiCams){
			foundInMono = Util::isInFrame(threeLoc);
		}
		foundInBoth = Util::isInBothFrames(leftLoc, rightLoc);
	    
		if (foundInBoth){
			CoordinateReal real = stereo.getLocation(leftLoc, rightLoc);
			//print the current location
			cout << "x: " << real.x() << "y: " << real.y() << "z: " << real.z() << endl;
			//cout << "time in seconds" << float(clock() - beginTime) / CLOCKS_PER_SEC << endl;
			if (!found){
				cout << "initialising kalman filter" << endl;
				kalman.initialise(real);
			}
			else {
				kalman.stereoObservation(real);
			}
			 
			double curTime = double(clock())/CLOCKS_PER_SEC;
			cout << "curTime" << curTime << endl;
			stat.getVel(real, curTime);
			foundInBoth = false;
			found = true;
		}
		if (foundInMono){
			// pass the observation 
			kalman.observation(threeLoc, three);
			foundInMono = false;
		}
		if (cross){
			// add cross to all the frames
			line(frameRight, horizontalOne, horizontalTwo, cv::Scalar(0, 255, 0), 2); 
			line(frameRight, verticalOne, verticalTwo, cv::Scalar(0, 0, 255), 2);
			line(frameLeft, horizontalOne, horizontalTwo, cv::Scalar(0, 255, 0), 2);
			line(frameLeft, verticalOne, verticalTwo, cv::Scalar(0, 0, 255), 2);
			//multi cam
			if (multiCams){
				line(frameThree, horizontalOne, horizontalTwo, cv::Scalar(0, 255, 0), 2);
				line(frameThree, verticalOne, verticalTwo, cv::Scalar(0, 0, 255), 2);
			}
		}
		cv::imshow("left", frameLeft);
		cv::imshow("right", frameRight);
		if (multiCams){
			cv::imshow("mid", frameThree);
		}
		command = waitKey(1);
		if (surfing){
			waitKey(0);
			surfing = false;
		}
		clock_t end = clock();
		elapsedTime = double(end - beginTime) / CLOCKS_PER_SEC;
		waitDelta = framesPerSecond - elapsedTime;
		if (waitDelta > 0){
			command = waitKey(waitDelta* 1000);
		}
		 end = clock();
		elapsedTime = double(end - beginTime) / CLOCKS_PER_SEC;
		cout << "fps"  << 1 / elapsedTime << endl;
		//convert fps to string
		string fps = std::to_string(1 / elapsedTime);
		fps += "\n";
		framesFile_ << fps;

	}
	framesFile_.close();
	kalman.closeFile();
	return;
}

void camCount(char c){
	switch (c){
	case '3':
		//multiple cameras
		multiCams = true;
		break;
	}
}

void commands(char c){
	switch (c){
	case 'm':
		multiCams = true;
		break;
	case 't':
		threeDebug = !threeDebug;
		break;
	case 'c':
		cross = !cross;
		break;
	case 'h':
		hsv = true;
		//simulation = true;
		break;
	case 'o':
		record = true;
		//simulation = true;
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