#ifndef PF_H__
#define PF_H__

#include <vector>
#include <sstream>
#include <random>

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "mcl/LikelihoodFieldMap.h"
using namespace std;

class Pose
{
public:
	Pose(double x, double y, double t)
	{
		set(x, y, t);
	}

	void set(double x, double y, double t)
	{
		x_ = x;
		y_ = y;
		t_ = t;
	}

	void set(const Pose &p)
	{
		x_ = p.x_;
		y_ = p.y_;
		t_ = p.t_;
	}

	string to_s(void)
	{
		stringstream s;
		s << "x:" << x_ << "\ty:" << y_ << "\tt:" << t_;
		return s.str();
	}

	double x_, y_, t_;
};

class Particle
{
public:
	Particle(double x, double y, double t, double w);

	double likelihood(LikelihoodFieldMap &map, const Scan &scan);
	Pose p_;
	double w_;
};

class OdomError
{
public:
	OdomError(double ff, double fr, double rf, double rr);
	void setDev(double length, double angle);
	double drawFwNoise(void);
	double drawRotNoise(void);
private:
	double fw_dev_;
	double rot_dev_;

	double fw_var_per_fw_;
	double fw_var_per_rot_;
	double rot_var_per_fw_;
	double rot_var_per_rot_;

	std::normal_distribution<> std_norm_dist_;
	
	std::random_device seed_gen_;
	std::default_random_engine engine_;
};

/*
class LikelihoodFieldMap
{
public: 
	LikelihoodFieldMap(const nav_msgs::OccupancyGrid &map);
	~LikelihoodFieldMap();

	//cellToPos(int x, int y);
	bool *posToCell(double x, double y);
	int posToCellX(double x);
	int posToCellY(double y);

	double likelihood(double x, double y);

	void setLikelihood(int x, int y);

	vector<bool *> cells_;
	vector<double *> likelihoods_;
	int width_;
	int height_;

	double resolution_;
	double origin_x_;
	double origin_y_;
};

class Scan
{
public: 
	int seq_;
	int processed_seq_;
	double angle_max_;
	double angle_min_;
	double angle_increment_;
	vector<double> ranges_;
};
*/

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
	void meanPose(double &x_mean, double &y_mean, double &t_mean,
			double &x_var, double &y_var, double &t_var,
			double &xy_cov, double &yt_cov, double &tx_cov);

	void setScan(const sensor_msgs::LaserScan::ConstPtr &msg);
private:
	Pose *last_odom_;
	Pose *prev_odom_;

	Scan scan_;

	double normalizeAngle(double t);
	void resampling(void);

	OdomError odom_error_;

	LikelihoodFieldMap map_;
};

#endif
