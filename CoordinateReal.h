#pragma once
class CoordinateReal
{
public:
	CoordinateReal(double x, double y, double z);
	CoordinateReal();
	double x() { return x_; }
	double y() { return y_; }
	double z() { return z_; }
	double setZ(double x){ x_ = x; }
	double setY(double y){ y_ = y; }
	double setX(double z){ z_ = z; }
	~CoordinateReal();

private:
	double x_;
	double y_;
	double z_;

};

