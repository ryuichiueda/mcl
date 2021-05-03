#ifndef PARTICLE_H__
#define PARTICLE_H__

#include "mcl/Pose.h"
#include "mcl/LikelihoodFieldMap.h"
using namespace std;

class Particle
{
public:
	Particle(double x, double y, double t, double w);

	double likelihood(LikelihoodFieldMap *map, const Scan &scan);
	Pose p_;
	double w_;
};

#endif
