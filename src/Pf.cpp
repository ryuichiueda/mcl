#include "Pf.h"
#include <stdlib.h>

Particle::Particle()
{
	x = (double)rand()/RAND_MAX;
	y = (double)rand()/RAND_MAX;
	t = (double)rand()/RAND_MAX;
	w = 1.0;
}

Pf::Pf(){}
Pf::~Pf(){}
