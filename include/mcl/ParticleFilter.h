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
			double ff, double fr, double rf, double rr,
			const nav_msgs::OccupancyGrid &map);
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

	OdomModel odom_model_;
	LikelihoodFieldMap map_;
};

#endif
