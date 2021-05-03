#include "mcl/OccupancyGridMap.h"

OccupancyGridMap::OccupancyGridMap(const nav_msgs::OccupancyGrid &map)
{
/*	ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
		resp.map.info.width, resp.map.info.height, resp.map.info.resolution);
		*/

	width_ = map.info.width;
	height_ = map.info.height;
	origin_x_ = map.info.origin.position.x;
	origin_y_ = map.info.origin.position.y;

	resolution_ = map.info.resolution;

	for(int x=0; x<width_; x++){
		cells_.push_back(new bool[height_]);
		likelihoods_.push_back(new double[height_]);
		for(int y=0; y<height_; y++){
			cells_[x][y] = map.data[x + y*width_] > 50;
			likelihoods_[x][y] = 0.0;
		}
	}

	for(int x=0; x<width_; x++){
		for(int y=0; y<height_; y++){
			setLikelihood(x, y);
		}
	}

	/*
	for(int y=0; y<map.info.height; y++){
		for(int x=0; x<map.info.width; x++){
			putchar(cells_[x][y] ? '*' : ' ');
		}
		putchar('\n');
	}
	*/
}

OccupancyGridMap::~OccupancyGridMap()
{
	for(auto &c : cells_)
		delete [] c;

	for(auto &c : likelihoods_)
		delete [] c;
}


double OccupancyGridMap::likelihood(double x, double y)
{
	int ix = (int)floor((x - origin_x_)/resolution_);
	int iy = (int)floor((y - origin_y_)/resolution_);

	if(ix < 0 or iy < 0)
		return 0.0;
	if(ix >= width_ or iy >= height_)
		return 0.0;

	return likelihoods_[ix][iy];
}

bool *OccupancyGridMap::posToCell(double x, double y)
{
	int ix = (int)floor((x - origin_x_)/resolution_);
	int iy = (int)floor((y - origin_y_)/resolution_);

	if(ix < 0 or iy < 0)
		return NULL;
	if(ix >= width_ or iy >= height_)
		return NULL;

	return &cells_[ix][iy];
}


int OccupancyGridMap::posToCellX(double x)
{
	int ix = (int)floor((x - origin_x_)/resolution_);

	if(ix < 0 or ix >= width_)
		return -1;

	return ix;
}

int OccupancyGridMap::posToCellY(double y)
{
	int iy = (int)floor((y - origin_y_)/resolution_);

	if(iy < 0 or iy >= height_)
		return -1;

	return iy;
}

void OccupancyGridMap::setLikelihood(int x, int y){
	if(not cells_[x][y])
		return;

	for(int i=-2+x;i<=2+x;i++){
		if(i < 0 or i >= width_)
			continue;

		for(int j=-2+y;j<=2+y;j++){
			if(j < 0 or j >= height_)
				continue;

			likelihoods_[i][j] += 0.25;
		}
	}

	for(int i=-1+x;i<=1+x;i++){
		if(i < 0 or i >= width_)
			continue;

		for(int j=-1+y;j<=1+y;j++){
			if(j < 0 or j >= height_)
				continue;

			likelihoods_[i][j] += 0.25;
		}
	}

	likelihoods_[x][y] += 0.25;
}
