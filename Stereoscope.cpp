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
	distance_ = 0;
	string curLine;
}


Stereoscope::~Stereoscope()
{

}

double Stereoscope::getDistance(CoordinateReal leftImage, CoordinateReal rightImage){

	double z = 0;
	int deltaOne = leftImage.x() - halfPixelWidth_;
	double radiansForFov = convertToRadians(halfFOV_);
	//the right of centre line of left image is positive
	double tanOmega1 = (deltaOne * tan(radiansForFov)) / halfPixelWidth_;
	int deltaTwo = halfPixelWidth_ - rightImage.x() ;
	//the left of centre line of left image is positive
	double tanOmega2 = (deltaTwo * tan(radiansForFov)) / halfPixelWidth_;
	//guess the depth of the object 
	z = distanceApart_ / (tanOmega1 + tanOmega2) ;
	currentLocation_.setZ(z);
	// get the angle in ther vertical direction from the centre
	
	// the zero of x will be defined by the average co-ordinnate of both cameras
	
	return z;
}

void Stereoscope::calculateZ(double theta, double distance){

}

double Stereoscope::convertToRadians(double degrees){
	//radians = degrees * pi/180
	double radians = degrees * (PI / 180);
	cout << "degrees: " << degrees << " radians: " << radians<< endl;
	return radians;
}

