#include "mcl/LikelihoodFieldMap.h"

LikelihoodFieldMap::LikelihoodFieldMap(const nav_msgs::OccupancyGrid &map)
{
	width_ = map.info.width;
	height_ = map.info.height;

	origin_x_ = map.info.origin.position.x;
	origin_y_ = map.info.origin.position.y;

	resolution_ = map.info.resolution;

	for(int x=0; x<width_; x++){
		likelihoods_.push_back(new double[height_]);

		for(int y=0; y<height_; y++)
			likelihoods_[x][y] = 0.0;
	}

	for(int x=0; x<width_; x++)
		for(int y=0; y<height_; y++)
			if(map.data[x + y*width_] > 50)
				setLikelihood(x, y);
}

LikelihoodFieldMap::LikelihoodFieldMap(const LikelihoodFieldMap &map)
{
	width_ = map.width_;
	height_ = map.height_;

	origin_x_ = map.origin_x_;
	origin_y_ = map.origin_x_;

	resolution_ = map.resolution_;

	for(int x=0; x<width_; x++){
		likelihoods_.push_back(new double[height_]);

		for(int y=0; y<height_; y++)
			likelihoods_[x][y] = map.likelihoods_[x][y];
	}
}

LikelihoodFieldMap::~LikelihoodFieldMap()
{
	for(auto &c : likelihoods_)
		delete [] c;
}


double LikelihoodFieldMap::likelihood(double x, double y)
{
	int ix = (int)floor((x - origin_x_)/resolution_);
	int iy = (int)floor((y - origin_y_)/resolution_);

	if(ix < 0 or iy < 0)
		return 0.0;
	if(ix >= width_ or iy >= height_)
		return 0.0;

	return likelihoods_[ix][iy];
}

void LikelihoodFieldMap::setLikelihood(int x, int y){
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
