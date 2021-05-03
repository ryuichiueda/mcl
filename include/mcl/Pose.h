#ifndef POSE_H__
#define POSE_H__

#include <sstream>

using namespace std;

class Pose
{
public:
	Pose(double x, double y, double t)
	{
		set(x, y, t);
	}

	void set(double x, double y, double t)
	{
		x_ = x;
		y_ = y;
		t_ = t;
	}

	void set(const Pose &p)
	{
		x_ = p.x_;
		y_ = p.y_;
		t_ = p.t_;
	}

	string to_s(void)
	{
		stringstream s;
		s << "x:" << x_ << "\ty:" << y_ << "\tt:" << t_;
		return s.str();
	}

	double x_, y_, t_;
};

#endif
