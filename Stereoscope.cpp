#include "Stereoscope.h"
#include "Camera.h"
#include <iostream>
#include <fstream>
#include <string> 
#include <math.h>

#define PI 3.1415926
using namespace std;
Stereoscope::Stereoscope()
{
	//set up the variables for calculation;
	halfFOV_ = 24.2;
	halfPixelWidth_ = Camera::reso_width / 2;
	string filePath = "../../../../ThesisImages/";
	distance_ = 0;
	string curLine;
	//load the images from file;
	ifstream file(filePath + "images.txt");
	if (!file.is_open()){
		cout << "cantFind" << endl;
	}
	while (getline(file, curLine)){
		cout << curLine << endl;
		Mat temp = imread(filePath + curLine, CV_LOAD_IMAGE_COLOR);
		targetImages_.push_back(temp);
	}
	cout << targetImages_.size() << " images loaded" << endl;
}


Stereoscope::~Stereoscope()
{

}

void Stereoscope::displayImages(){
	for (int count = 0; count < targetImages_.size(); count++){
		if (count = 0){
			imwrite("../../../../ThesisImages/oneResult.jpg", targetImages_[count]);
		}
		else{
			imwrite("../../../../ThesisImages/twoResult.jpg", targetImages_[count]);
		}

	}
}

double Stereoscope::getDistance(){
	for (int count = 0; count < targetImages_.size(); count++){

		Mat temp = blurPicture(targetImages_[count]);
		greyImages_.push_back(temp);
		Point center = findCircleCentre(greyImages_[count], targetImages_[count]);
		centers_.push_back(center);
	}
	//check the values 
	for (int count = 0; count < centers_.size(); count++){
		Point circleCenter = centers_[count];
		cout << "x: " << circleCenter.x << "y: " << circleCenter.y << endl;
	}
	// plug values into the formula 
	 
	//get tanOmega1 = delta 1 (distance from center of camera * tan(fov/2)  / (xpixels/2)
	int deltaOne = centers_[1].x - halfPixelWidth_;
	double radiansForFov = convertToRadians(halfFOV_);
	double tanOmega1 = (deltaOne * tan(radiansForFov)) / halfPixelWidth_;
	int deltaTwo = halfPixelWidth_ -  centers_[0].x;
	double tanOmega2 = (deltaTwo * tan(radiansForFov)) / halfPixelWidth_;
	double distance = distanceApart_ / (tanOmega1 + tanOmega2)  + halfWidthBall_;
	//displayImages();
	return distance;
}

double Stereoscope::convertToRadians(double degrees){
	//radians = degrees * pi/180
	double radians = degrees * (PI / 180);
	cout << "degrees: " << degrees << " radians: " << radians<< endl;
	return radians;
}

Point Stereoscope::findCircleCentre(cv::Mat searchSpace, cv::Mat original){
	Point center;
	vector<Vec3f> circles;
	circles.clear();
	//imshow("searching:", searchSpace);
	waitKey(3);
	HoughCircles(searchSpace, circles, CV_HOUGH_GRADIENT, 2, searchSpace.rows/8, 200, 120, 0, 0);
	cout << searchSpace.rows/8 << endl;
	cout << "size: " << circles.size() << endl;
	for (int i = 0; i < circles.size(); i++)
	{
		cout << "searching" << endl;
		center = Point(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle(original, center, 3, Scalar(0, 0, 0), 10, 8, 0);
		// circle outline
		circle(original, center, radius, Scalar(0, 0, 255), 3, 8, 0);
		imshow("what", original);
		waitKey(3);
	}
	
	return center;
}

Mat Stereoscope::blurPicture(Mat imageToBlur){
	//convert the image to gray scale (performance issue)
	cvtColor(imageToBlur, imageToBlur, CV_BGR2GRAY);
	// place a gaussian blue on the whole image to remove noise. 
	GaussianBlur(imageToBlur, imageToBlur, Size(9, 9), 2, 2);
	imwrite("../../../../ThesisImages/blurredImage.jpg", imageToBlur);
	return imageToBlur;
}

