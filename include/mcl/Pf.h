#ifndef PF_H__
#define PF_H__

class Pose
{
public:
	Pose()
	{
		set(0.0, 0.0, 0.0);
	}

	void set(double x, double y, double t)
	{
		x_ = x;
		y_ = y;
		t_ = t;
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

	Particle particles[100];
	void updateOdom(double x, double y, double t);
private:
};

#endif
