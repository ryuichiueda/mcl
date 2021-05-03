#include "mcl/Particle.h"
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
using namespace std;

Particle::Particle(double x, double y, double t, double w) : p_(x, y, t)
{
	w_ = w;
}

double Particle::likelihood(LikelihoodFieldMap &map, const Scan &scan)
{
	double ans = 0.0;
	for(int i=0;i<scan.ranges_.size();i++){
		if(isinf(scan.ranges_[i]))
			continue;

		double ang = scan.angle_min_ + i*scan.angle_increment_;
		double lx = p_.x_ + scan.ranges_[i] * cos(ang + p_.t_);
		double ly = p_.y_ + scan.ranges_[i] * sin(ang + p_.t_);

		ans += map.likelihood(lx, ly);
	}
	return ans;
}
