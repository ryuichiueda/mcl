#ifndef PF_H__
#define PF_H__

#include <vector>
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
private:
	Pose last_odom_;
	Pose prev_odom_;
};

#endif
