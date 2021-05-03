#include "mcl/ParticleFilter.h"
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
using namespace std;

ParticleFilter::ParticleFilter(double x, double y, double t, int num, 
				double ff, double fr, double rf, double rr,
				const nav_msgs::OccupancyGrid &map) 
	: odom_model_(ff, fr, rf, rr), last_odom_(NULL), prev_odom_(NULL), map_(map)
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

void ParticleFilter::resampling(void)
{
	vector<double> accum;
	accum.push_back(particles_[0].w_);
	for(int i=1;i<particles_.size();i++){
		accum.push_back(accum.back() + particles_[i].w_);
	}

	vector<Particle> old;
	for(auto p : particles_){
		old.push_back(p);
	}

	double start = (double)rand()/(RAND_MAX * particles_.size());
	double step = 1.0/particles_.size();

	vector<int> chosen;

	int tick = 0;
	for(int i=0; i<particles_.size(); i++){
		while(accum[tick] <= start + i*step){
			tick++;
			if(tick == particles_.size()){
				ROS_ERROR("RESAMPLING FAILED");
				exit(1);
			}	
		}	
		chosen.push_back(tick);
	}

	for(int i=0; i<particles_.size(); i++){
		particles_[i].p_.x_ = old[chosen[i]].p_.x_;
		particles_[i].p_.y_ = old[chosen[i]].p_.y_;
		particles_[i].p_.t_ = old[chosen[i]].p_.t_;
		particles_[i].w_ = old[chosen[i]].w_;
	}
}

void ParticleFilter::updateSensor(void)
{
	if(scan_.processed_seq_ == scan_.seq_)
		return;

	vector<double> ranges;
	int seq = -1;
	/* copy scan ranges safely */
	while(seq != scan_.seq_){
		ranges.clear();
		seq = scan_.seq_;
		copy(scan_.ranges_.begin(), scan_.ranges_.end(), back_inserter(ranges) );
	}

	for(auto &p : particles_)
		p.w_ *= p.likelihood(map_, scan_);

	double sum = 0.0;
	for(const auto &p : particles_)
		sum += p.w_;

	if(sum < 0.000000000001){
		for(auto &p : particles_)
			p.w_ = 1.0/particles_.size();
		
	}else{
		for(auto &p : particles_)
			p.w_ /= sum;

		resampling();
	}

	scan_.processed_seq_ = scan_.seq_;
}

void ParticleFilter::updateOdom(double x, double y, double t)
{
	if(last_odom_ == NULL){
		last_odom_ = new Pose(x, y, t);
		prev_odom_ = new Pose(x, y, t);
		return;
	}else
		last_odom_->set(x, y, t);

	double dx = x - prev_odom_->x_;
	double dy = y - prev_odom_->y_;
	double dt = normalizeAngle(t - prev_odom_->t_);

	if(fabs(dx) < 0.001 and fabs(dy) < 0.001 and fabs(dt) < 0.001)
		return;

	double fw_length = sqrt(dx*dx + dy*dy);
	double fw_direction = normalizeAngle((fw_length > 0.000001 ? atan2(dy, dx) : 0.0) - prev_odom_->t_);

	odom_model_.setDev(fw_length, dt);

	for(auto &p : particles_){
		double rot_noise = odom_model_.drawRotNoise();

		double fw_length_e = fw_length + odom_model_.drawFwNoise();
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

