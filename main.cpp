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
	Camera one(0);
	Camera two(2);
	FeatureExtraction surf(3000);
	Stereoscope stereo;
	bool running = true;
	char curPressed = ' ';
	surf.addImageToLib("backToTheFutureCover.jpg");
	while (running){
		curPressed = waitKey(10);
		//////left frame =============================
		cv::Mat frameLeft = one.grabFrame();
	
		cv::Mat frameRight = two.grabFrame();

		frameRight = two.grabFrame();
		if (curPressed == ' '){
			//char pressedKey = 'p';
			////
			std::vector<CoordinateReal> coordLeft = surf.detect(frameLeft, true);
			if (!coordLeft.empty()){
				int thickness = -1;
				int lineType = 8;
				cv::circle(frameLeft, cv::Point2f(coordLeft[0].x(), coordLeft[0].y()), 5,
					cv::Scalar(0, 0, 255),
					thickness,
					lineType);
			}
			////right frame ==================================


			std::vector<CoordinateReal> coordRight = surf.detect(frameRight, true);
			if (!coordRight.empty()){
				int thickness = -1;
				int lineType = 8;
				cv::circle(frameRight, cv::Point2f(coordRight[0].x(), coordRight[0].y()), 5,
					cv::Scalar(0, 0, 255),
					thickness,
					lineType);
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
		}
		imshow("left", frameLeft);
		imshow("right", frameRight);
		
	}

}