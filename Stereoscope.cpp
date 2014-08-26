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

CoordinateReal Stereoscope::getLocation(CoordinateReal leftImage, CoordinateReal rightImage){

	double z = 0;
	double x = 0;
	double y = 0;
	int deltaOne = leftImage.x() - halfPixelWidth_;
	double radiansForFov = convertToRadians(halfFOV_);
	//the right of centre line of left image is positive
	double tanOmega1 = (deltaOne * tan(radiansForFov)) / halfPixelWidth_;
	int deltaTwo =  rightImage.x() - halfPixelWidth_;
	//the left of centre line of left image is positive
	double tanOmega2 = (deltaTwo * tan(radiansForFov)) / halfPixelWidth_;
	//guess the depth of the object 
	z = distanceApart_ / (tanOmega1 - tanOmega2) ;
	currentLocation_.setZ(z);
	// get the angle in ther vertical direction from the centre

	// the zero of x will be defined by the average co-ordinnate of both cameras
	x = calculateX(z, tanOmega1, tanOmega2);
	y = calculateY(leftImage, rightImage, z, x);
	currentLocation_.setX(x);
	return currentLocation_;
}

double Stereoscope::calculateX(double distance, double tanOmegaLeft, double tanOmegaRight){
	double calculatedX = 0;
	//left alpha = deltaXLeft - b/2
	float leftDeltaX = distance * tanOmegaLeft;
	float leftX = leftDeltaX - distanceApart_ / 2;
	//right alpha = b/2 + deltaXRight
	float rightDeltaX = distance * tanOmegaRight;
	float rightX = distanceApart_ / 2 - rightDeltaX;
	calculatedX = (leftX + rightX) / 2;
	//return averageX;
	return calculatedX;
}

float Stereoscope::calculateY(CoordinateReal leftImage, CoordinateReal rightImage, double distance, double x){
	//get the hypotenuse of the z and x
	float averageY = (leftImage.y() + rightImage.y()) / 2;
	float angle = atan(distance / x);
	angle = std::abs(angle);
	float hypotenuse = distance / cos(angle);
	//figure out the distance in the y direction according to the hypotenuse ( abs dist) and height
	float maxHeight = hypotenuse * tan(vertFov_ / 2);
	//now figure out the ration between half vertical resolution and the pixel height of th eobject
	float ratio = averageY / Camera::reso_height / 2;
	float y = maxHeight * ratio;
	return y;
}

double Stereoscope::convertToRadians(double degrees){
	//radians = degrees * pi/180
	double radians = degrees * (PI / 180);
	cout << "degrees: " << degrees << " radians: " << radians<< endl;
	return radians;
}

