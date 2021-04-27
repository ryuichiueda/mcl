#include "mcl/Pf.h"
#include <iostream>
#include <stdlib.h>
#include <cmath>
using namespace std;

Particle::Particle()
{
}

ParticleFilter::ParticleFilter()
{
	Particle p;
	for(int i=0;i<100;i++)
		particles_.push_back(p);
}

ParticleFilter::~ParticleFilter(){}

void ParticleFilter::updateOdom(double x, double y, double t)
{
	last_odom_.set(x, y, t);

	/* These lines should be refactored. */
	double dx = x - prev_odom_.x_;
	double dy = y - prev_odom_.y_;
	double move_length = sqrt(dx*dx + dy*dy);
	double move_direction = (move_length > 0.000001 ? atan2(dy, dx) : 0.0) - prev_odom_.t_;
	double dt = t - prev_odom_.t_;

	if(fabs(dx) < 0.001 and fabs(dy) < 0.001 and fabs(dt) < 0.001)
		return;

	/*
	for(auto &p : particles_){
		double ang = move_direction + p.p_.t_;

		double ang_e = ang + 3.141592*18*((double)rand()/RAND_MAX - 0.5);
		double move_length_e = move_length + 0.1*((double)rand()/RAND_MAX - 0.5);
		double dt_e = dt + 3.141592*18*((double)rand()/RAND_MAX - 0.5);

		p.p_.x_ += move_length_e*cos(ang_e);
		p.p_.y_ += move_length_e*sin(ang_e);
		p.p_.t_ += dt_e;
	}
	*/

	prev_odom_.set(last_odom_);
}
