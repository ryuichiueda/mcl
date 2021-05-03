/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef PF_H__
#define PF_H__

#include <vector>
#include <sstream>
#include <random>

#include "mcl/Particle.h"
#include "mcl/OdomModel.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "mcl/LikelihoodFieldMap.h"
using namespace std;

class ParticleFilter
{
public: 
	ParticleFilter(double x, double y, double t, int num,
			const shared_ptr<OdomModel> &odom_model,
			const shared_ptr<LikelihoodFieldMap> &map);
	~ParticleFilter();

	vector<Particle> particles_;
	void updateSensor(void);
	void updateOdom(double x, double y, double t);

	void setScan(const sensor_msgs::LaserScan::ConstPtr &msg);
	void meanPose(double &x_mean, double &y_mean, double &t_mean,
			double &x_var, double &y_var, double &t_var,
			double &xy_cov, double &yt_cov, double &tx_cov);
private:
	Pose *last_odom_;
	Pose *prev_odom_;

	Scan scan_;

	double normalizeAngle(double t);
	void resampling(void);
	bool normalize(void);
	void resetWeight(void);

	shared_ptr<OdomModel> odom_model_;
	shared_ptr<LikelihoodFieldMap> map_;
};

#endif
