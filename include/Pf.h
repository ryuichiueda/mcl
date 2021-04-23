#ifndef PF_H__
#define PF_H__

class Particle
{
public:
	Particle();

	double x;
	double y;
	double t;
	double w;
};

class Pf
{
public: 
	Pf();
	~Pf();

	Particle particles[100];
	void setVelocity(double vel, double rot);
};

#endif
