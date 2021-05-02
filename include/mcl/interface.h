/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some parts are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef INTERFACE_H__
#define INTERFACE_H__

#include <ros/ros.h>
#include "mcl/pf.h"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class MclNode
{
public:
	MclNode();
	~MclNode();

	void loop(void);
private:
	ParticleFilter *pf_;
	ros::NodeHandle nh_;

	ros::Publisher particlecloud_pub_;
	ros::Publisher pose_pub_;

	std::string base_frame_id_;
	std::string global_frame_id_;
	std::string odom_frame_id_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
	std::shared_ptr<tf2_ros::TransformListener> tfl_;
	std::shared_ptr<tf2_ros::Buffer> tf_;

	tf2::Transform latest_tf_;

	//geometry_msgs::PoseStamped latest_odom_pose_;

	void publishPose(double x, double y, double t,
			double x_dev, double y_dev, double t_dev,
			double xy_cov, double yt_cov, double tx_cov);
	void publishOdomFrame(double x, double y, double t);
	void publishParticles(void);
	void sendTf(void);
	bool getOdomPose(double& x, double& y, double& yaw);
};

#endif
