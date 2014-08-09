#pragma once
class CoordinateReal
{
public:
	CoordinateReal(double x, double y, double z);
	double x() { return x_; }
	double y() { return y_; }
	double z() { return z_; }
	~CoordinateReal();

private:
	double x_;
	double y_;
	double z_;

};

