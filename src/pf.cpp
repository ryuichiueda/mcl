#include "mcl/pf.h"
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
using namespace std;


OccupancyGridMap::OccupancyGridMap(const nav_msgs::OccupancyGrid &map)
{
/*	ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
		resp.map.info.width, resp.map.info.height, resp.map.info.resolution);
		*/

	width_ = map.info.width;
	height_ = map.info.height;

	resolution_ = map.info.resolution;

	for(int x=0; x<width_; x++){
		cells_.push_back(new bool[height_]);
		for(int y=0; y<height_; y++){
			cells_[x][y] = map.data[x + y*width_] > 50;
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
		delete c;
}

bool *OccupancyGridMap::posToCell(double x, double y)
{
	double shift_x = x + resolution_*width_/2;
	double shift_y = y + resolution_*height_/2;

	int ix = (int)floor(shift_x/resolution_);
	int iy = (int)floor(shift_y/resolution_);

	if(ix < 0 or iy < 0)
		return NULL;
	if(ix >= width_ or iy >= height_)
		return NULL;

	return &cells_[ix][iy];
}

int OccupancyGridMap::posToCellX(double x)
{
	double shift_x = x + resolution_*width_/2;
	int ix = (int)floor(shift_x/resolution_);

	if(ix < 0 or ix >= width_)
		return -1;

	return ix;
}

int OccupancyGridMap::posToCellY(double y)
{
	double shift_y = y + resolution_*height_/2;
	int iy = (int)floor(shift_y/resolution_);

	if(iy < 0 or iy >= height_)
		return -1;

	return iy;
}

OdomError::OdomError(double ff, double fr, double rf, double rr) 
	: std_norm_dist_(0.0, 1.0), fw_dev_(0.0), rot_dev_(0.0), engine_(seed_gen_())
{
	fw_var_per_fw_ = ff*ff;
	fw_var_per_rot_ = fr*fr;
	rot_var_per_fw_ = rf*rf;
	rot_var_per_rot_ = rr*rr;
}

void OdomError::setDev(double length, double angle)
{
	fw_dev_ = sqrt( fabs(length)*fw_var_per_fw_ + fabs(angle)*fw_var_per_rot_ );
	rot_dev_ = sqrt( fabs(length)*rot_var_per_fw_ + fabs(angle)*rot_var_per_rot_ );
}

double OdomError::drawFwNoise(void)
{
	return std_norm_dist_(engine_) * fw_dev_;
}

double OdomError::drawRotNoise(void)
{
	return std_norm_dist_(engine_) * rot_dev_;
}

Particle::Particle(double x, double y, double t, double w) : p_(x, y, t)
{
	w_ = w;
}

ParticleFilter::ParticleFilter(double x, double y, double t, int num, 
				double ff, double fr, double rf, double rr,
				const nav_msgs::OccupancyGrid &map) 
	: odom_error_(ff, fr, rf, rr), last_odom_(NULL), prev_odom_(NULL), map_(map)
{
	if(num <= 0)
		ROS_ERROR("NO PARTICLE");

	Particle p(x, y, t, 1.0/num);
	for(int i=0; i<num; i++)
		particles_.push_back(p);

	scan_.processed_seq_ = -1;
}

ParticleFilter::~ParticleFilter()
{
	delete last_odom_;
	delete prev_odom_;
}

void ParticleFilter::updateSensor(void)
{
	if(scan_.processed_seq_ == scan_.seq_){
		ROS_INFO("SKIP");
		return;
	}

	vector<double> ranges;
	int seq = -1;
	/* copy scan ranges safely */
	while(seq != scan_.seq_){
		ranges.clear();
		seq = scan_.seq_;
		copy(scan_.ranges_.begin(), scan_.ranges_.end(), back_inserter(ranges) );
	}

	/* change weight */
	vector<bool *> cells_;
	int r = (int)floor((double)particles_.size()*rand()/RAND_MAX);
	double px = particles_[r].p_.x_;
	double py = particles_[r].p_.y_;
	double pt = particles_[r].p_.t_;

	for(int i=0;i<scan_.ranges_.size();i++){
		if(isinf(scan_.ranges_[i]))
			continue;

		double ang = scan_.angle_min_ + i*scan_.angle_increment_;
		double lx = px + scan_.ranges_[i] * cos(ang + pt);
		double ly = py + scan_.ranges_[i] * sin(ang + pt);

		int ix = map_.posToCellX(lx);
		int iy = map_.posToCellY(ly);

		/*
		if(ix >= 0 and iy >= 0)
			map_.cells_[ix][iy] = false;
			*/

		bool *c = map_.posToCell(lx, ly);
		if(c != NULL)
			*c = true;

		ROS_INFO("laser: %f, %f, %d, %d", lx, ly, ix, iy);
	}

	for(int y=0;y<map_.height_;y++){
		for(int x=0;x<map_.width_;x++)
			putchar( map_.cells_[x][y] ? '*' : ' ');
		putchar('\n');
	}
	cout << endl;

	scan_.processed_seq_ = scan_.seq_;
}

void ParticleFilter::updateOdom(double x, double y, double t)
{
	if(last_odom_ == NULL){
		last_odom_ = new Pose(x, y, t);
		prev_odom_ = new Pose(x, y, t);
	}else
		last_odom_->set(x, y, t);

	double dx = x - prev_odom_->x_;
	double dy = y - prev_odom_->y_;
	double dt = normalizeAngle(t - prev_odom_->t_);

	if(fabs(dx) < 0.001 and fabs(dy) < 0.001 and fabs(dt) < 0.001)
		return;

	double fw_length = sqrt(dx*dx + dy*dy);
	double fw_direction = normalizeAngle((fw_length > 0.000001 ? atan2(dy, dx) : 0.0) - prev_odom_->t_);

	odom_error_.setDev(fw_length, dt);

	for(auto &p : particles_){
		double rot_noise = odom_error_.drawRotNoise();

		double fw_length_e = fw_length + odom_error_.drawRotNoise();
		double ang_e = fw_direction + p.p_.t_ + rot_noise;
		double dt_e = dt + rot_noise;

		p.p_.x_ += fw_length_e*cos(ang_e);
		p.p_.y_ += fw_length_e*sin(ang_e);
		p.p_.t_ = normalizeAngle(p.p_.t_ + dt_e);
	}

	prev_odom_->set(*last_odom_);
}

void ParticleFilter::meanPose(double &x_mean, double &y_mean, double &t_mean,
				double &x_dev, double &y_dev, double &t_dev,
				double &xy_cov, double &yt_cov, double &tx_cov)
{
	double x, y, t, t2;
	x = y = t = t2 = 0.0;
	for(const auto &p : particles_){
		x += p.p_.x_;
		y += p.p_.y_;
		t += p.p_.t_;
		t2 += normalizeAngle(p.p_.t_ + M_PI);
	}

	x_mean = x / particles_.size();
	y_mean = y / particles_.size();
	t_mean = t / particles_.size();
	double t2_mean = t2 / particles_.size();

	double xx, yy, tt, tt2;
	xx = yy = tt = tt2 = 0.0;
	for(const auto &p : particles_){
		xx += pow(p.p_.x_ - x_mean, 2);
		yy += pow(p.p_.y_ - y_mean, 2);
		tt += pow(p.p_.t_ - t_mean, 2);
		tt2 += pow(normalizeAngle(p.p_.t_ + M_PI) - t2_mean, 2);
	}

	if(tt > tt2){
		tt = tt2;
		t_mean = normalizeAngle(t2_mean - M_PI);
	}

	x_dev = xx/(particles_.size() - 1);
	y_dev = yy/(particles_.size() - 1);
	t_dev = tt/(particles_.size() - 1);

	double xy, yt, tx;
	xy = yt = tx = 0.0;
	for(const auto &p : particles_){
		xy += (p.p_.x_ - x_mean)*(p.p_.y_ - y_mean);
		yt += (p.p_.y_ - y_mean)*(normalizeAngle(p.p_.t_ - t_mean));
		tx += (p.p_.x_ - x_mean)*(normalizeAngle(p.p_.t_ - t_mean));
	}

	xy_cov = xy/(particles_.size() - 1);
	yt_cov = yt/(particles_.size() - 1);
	tx_cov = tx/(particles_.size() - 1);
}

double ParticleFilter::normalizeAngle(double t)
{
	while(t > M_PI)
		t -= 2*M_PI;
	while(t < -M_PI)
		t += 2*M_PI;

	return t;
}

void ParticleFilter::setScan(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	if(msg->ranges.size() != scan_.ranges_.size()){
		scan_.ranges_.resize(msg->ranges.size());
	}

	scan_.seq_ = msg->header.seq;
	for(int i=0; i<msg->ranges.size(); i++)
		scan_.ranges_[i] = msg->ranges[i];

	scan_.angle_min_ = msg->angle_min;
	scan_.angle_max_ = msg->angle_max;
	scan_.angle_increment_ = msg->angle_increment;

	/*
	cout << scan_.seq_ << endl;
	for(int i=0; i<scan_.ranges_.size(); i++)
		cout << scan_.ranges_[i] << " ";

	cout << endl;
	*/
}

