#include "CoordinateReal.h"


CoordinateReal::CoordinateReal(double x, double y, double z)
{
	x_ = x;
	y_ = y;
	z_ = z; 
}

CoordinateReal::CoordinateReal(){
	x_ = 0;
	y_ = 0;
	z_ = 0;
}

CoordinateReal::~CoordinateReal()
{
}

