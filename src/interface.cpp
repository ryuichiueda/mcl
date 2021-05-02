/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some parts are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include "mcl/interface.h"

#include "tf2/utils.h"
#include "geometry_msgs/PoseArray.h"

#include "nav_msgs/GetMap.h"
using namespace std;

MclNode::MclNode() : private_nh_("~") 
{
	initTF();
	initPF();

	particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
	pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("mcl_pose", 2, true);

	laser_scan_sub_ = nh_.subscribe("scan", 2, &MclNode::cbScan, this);
}

MclNode::~MclNode()
{
}

void MclNode::initTF(void)
{
	private_nh_.param("global_frame_id", global_frame_id_, string("map"));
	private_nh_.param("base_frame_id", base_frame_id_, string("base_footprint"));
	private_nh_.param("odom_frame_id", odom_frame_id_, string("odom"));

	tfb_.reset(new tf2_ros::TransformBroadcaster());
	tf_.reset(new tf2_ros::Buffer());
	tfl_.reset(new tf2_ros::TransformListener(*tf_));
}

void MclNode::initPF(void)
{
	double x, y, t;
	private_nh_.param("initial_pose_x", x, 0.0);
	private_nh_.param("initial_pose_y", y, 0.0);
	private_nh_.param("initial_pose_a", t, 0.0);

	double ff, fr, rf, rr;
	private_nh_.param("odom_fw_dev_per_fw_", ff, 0.0);
	private_nh_.param("odom_fw_dev_per_rot_", fr, 0.0);
	private_nh_.param("odom_rot_dev_per_fw_", rf, 0.0);
	private_nh_.param("odom_rot_dev_per_rot_", rr, 0.0);

	int num;
	private_nh_.param("num_particles", num, 0);

	nav_msgs::GetMap::Request  req;
	nav_msgs::GetMap::Response resp;
	ROS_INFO("Requesting the map...");
	while(!ros::service::call("static_map", req, resp)){
		ROS_WARN("Request for map failed; trying again...");
		ros::Duration d(0.5);
		d.sleep();
	}
	ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
		resp.map.info.width, resp.map.info.height, resp.map.info.resolution);

	pf_.reset(new ParticleFilter(x, y, t, num, ff, fr, rf, rr, resp.map));
}

void MclNode::cbScan(const sensor_msgs::LaserScan::ConstPtr &msg)
{
}

void MclNode::loop(void)
{
	double x, y, t;
	if(not getOdomPose(x, y, t)){
		ROS_INFO("can't get odometry info");
		return;
	}
	pf_->updateOdom(x, y, t);

	double x_var, y_var, t_var, xy_cov, yt_cov, tx_cov;
	pf_->meanPose(x, y, t, x_var, y_var, t_var, xy_cov, yt_cov, tx_cov);

	publishOdomFrame(x, y, t);
	publishPose(x, y, t, x_var, y_var, t_var, xy_cov, yt_cov, tx_cov);
	publishParticles();
}

void MclNode::publishPose(double x, double y, double t,
			double x_dev, double y_dev, double t_dev,
			double xy_cov, double yt_cov, double tx_cov)
{
	geometry_msgs::PoseWithCovarianceStamped p;
	p.header.frame_id = global_frame_id_;
	p.header.stamp = ros::Time::now();
	p.pose.pose.position.x = x;
	p.pose.pose.position.y = y;
	
	tf2::Quaternion q;
	q.setRPY(0, 0, t);
	tf2::convert(q, p.pose.pose.orientation);

	pose_pub_.publish(p);
}

void MclNode::publishOdomFrame(double x, double y, double t)
{
	geometry_msgs::PoseStamped odom_to_map;
	try{
		tf2::Quaternion q;
		q.setRPY(0, 0, t);
		tf2::Transform tmp_tf(q, tf2::Vector3(x, y, 0.0));
				
		geometry_msgs::PoseStamped tmp_tf_stamped;
		tmp_tf_stamped.header.frame_id = base_frame_id_;
		tmp_tf_stamped.header.stamp = ros::Time(0);
		tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);
		
		tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);

	}catch(tf2::TransformException){
		ROS_DEBUG("Failed to subtract base to odom transform");
		return;
	}
	tf2::convert(odom_to_map.pose, latest_tf_);
	
	ros::Time transform_expiration = (ros::Time(ros::Time::now().toSec() + 0.2));
	geometry_msgs::TransformStamped tmp_tf_stamped;
	tmp_tf_stamped.header.frame_id = global_frame_id_;
	tmp_tf_stamped.header.stamp = transform_expiration;
	tmp_tf_stamped.child_frame_id = odom_frame_id_;
	tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
	
	tfb_->sendTransform(tmp_tf_stamped);
}

void MclNode::publishParticles(void)
{
	geometry_msgs::PoseArray cloud_msg;
	cloud_msg.header.stamp = ros::Time::now();
	cloud_msg.header.frame_id = global_frame_id_;
	cloud_msg.poses.resize(pf_->particles_.size());

	for(int i=0;i<pf_->particles_.size();i++){		
		cloud_msg.poses[i].position.x = pf_->particles_[i].p_.x_;
		cloud_msg.poses[i].position.y = pf_->particles_[i].p_.y_;
		cloud_msg.poses[i].position.z = 0; 
		tf2::Quaternion q;
		q.setRPY(0, 0, pf_->particles_[i].p_.t_);
		tf2::convert(q, cloud_msg.poses[i].orientation);
	}		
	particlecloud_pub_.publish(cloud_msg);
}

bool MclNode::getOdomPose(double& x, double& y, double& yaw)
{
	geometry_msgs::PoseStamped ident;
	ident.header.frame_id = base_frame_id_;
	ident.header.stamp = ros::Time(0);
	tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
	
	geometry_msgs::PoseStamped odom_pose;
	try{
		this->tf_->transform(ident, odom_pose, odom_frame_id_);
	}catch(tf2::TransformException e){
    		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		return false;
	}
	x = odom_pose.pose.position.x;
	y = odom_pose.pose.position.y;
	yaw = tf2::getYaw(odom_pose.pose.orientation);

	return true;
}
