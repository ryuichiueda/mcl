#ifndef OCC_GRID_MAP_H__
#define OCC_GRID_MAP_H__

#include <vector>
#include "mcl/Scan.h"
#include "nav_msgs/OccupancyGrid.h"

class LikelihoodFieldMap
{
public: 
	LikelihoodFieldMap(const nav_msgs::OccupancyGrid &map);
	LikelihoodFieldMap(const LikelihoodFieldMap &map);
	~LikelihoodFieldMap();

	void setLikelihood(int x, int y);
	double likelihood(double x, double y);

	std::vector<double *> likelihoods_;
	int width_;
	int height_;

	double resolution_;
	double origin_x_;
	double origin_y_;
};

#endif
