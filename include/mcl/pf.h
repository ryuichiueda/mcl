#ifndef PF_H__
#define PF_H__

#include <vector>
#include <sstream>
using namespace std;

class Pose
{
public:
	Pose()
	{
		set(-2.0, -0.5, 0.0);
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

class Particle
{
public:
	Particle();

	Pose p_;
	double w_;
};

class ParticleFilter
{
public: 
	ParticleFilter();
	~ParticleFilter();

	vector<Particle> particles_;
	void updateOdom(double x, double y, double t);
	void meanPose(double &x_mean, double &y_mean, double &t_mean, double &x_dev, double &y_dev, double &t_dev);
private:
	Pose last_odom_;
	Pose prev_odom_;

	double normalizeAngle(double t);
};

#endif