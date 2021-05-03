#ifndef OCC_GRID_MAP_H__
#define OCC_GRID_MAP_H__

#include <vector>
#include "mcl/Scan.h"
#include "nav_msgs/OccupancyGrid.h"

class OccupancyGridMap
{
public: 
	OccupancyGridMap(const nav_msgs::OccupancyGrid &map);
	~OccupancyGridMap();

	//cellToPos(int x, int y);
	bool *posToCell(double x, double y);
	int posToCellX(double x);
	int posToCellY(double y);

	double likelihood(double x, double y);

	void setLikelihood(int x, int y);

	std::vector<bool *> cells_;
	std::vector<double *> likelihoods_;
	int width_;
	int height_;

	double resolution_;
	double origin_x_;
	double origin_y_;
};

#endif
