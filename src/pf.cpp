#include "mcl/pf.h"
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

	for(auto &p : particles_){
		double ang = move_direction + p.p_.t_;

		double ang_e = ang + 3.141592/36*((double)rand()/RAND_MAX - 0.5);
		double move_length_e = move_length + 0.01*((double)rand()/RAND_MAX - 0.5);
		double dt_e = dt + 3.141592/26*((double)rand()/RAND_MAX - 0.5);

		p.p_.x_ += move_length_e*cos(ang_e);
		p.p_.y_ += move_length_e*sin(ang_e);
		p.p_.t_ = normalizeAngle(p.p_.t_ + dt_e);
	}

	prev_odom_.set(last_odom_);
}

void ParticleFilter::meanPose(double &x_mean, double &y_mean, double &t_mean,
				double &x_dev, double &y_dev, double &t_dev)
{
	double x, y, t, t2;
	x = y = t = t2 = 0.0;
	for(const auto &p : particles_){
		x += p.p_.x_;
		y += p.p_.y_;
		t += p.p_.t_;
		t2 += normalizeAngle(p.p_.t_ + M_PI);
	}

	x_mean = x / particles_.size();
	y_mean = y / particles_.size();
	t_mean = t / particles_.size();
	double t2_mean = t2 / particles_.size();

	double xx, yy, tt, tt2;
	xx = yy = tt = tt2 = 0.0;
	for(const auto &p : particles_){
		xx += pow(p.p_.x_ - x_mean, 2);
		yy += pow(p.p_.y_ - y_mean, 2);
		tt += pow(p.p_.t_ - t_mean, 2);
		tt2 += pow(normalizeAngle(p.p_.t_ + M_PI) - t2_mean, 2);
	}

	if(tt > tt2){
		tt = tt2;
		t_mean = normalizeAngle(t2_mean - M_PI);
	}

	x_dev = sqrt(xx/(particles_.size() - 1));
	y_dev = sqrt(yy/(particles_.size() - 1));
	t_dev = sqrt(tt/(particles_.size() - 1));
}

double ParticleFilter::normalizeAngle(double t)
{
	while(t > M_PI)
		t -= 2*M_PI;
	while(t < -M_PI)
		t += 2*M_PI;

	return t;
}
