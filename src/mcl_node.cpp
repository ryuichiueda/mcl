/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some parts are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include <ros/ros.h>
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include "mcl/Pf.h"

//#include "tf2_ros/buffer.h"
//#include "tf2_ros/message_filter.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "tf2/LinearMath/Transform.h"


using namespace std;

class MclNode
{
public:
	MclNode()
	{
		pf_ = new ParticleFilter();
		global_frame_id_ = "map";
		base_frame_id_ = "base_link";
		odom_frame_id_ = "odom";

		tfb_.reset(new tf2_ros::TransformBroadcaster());
		tf_.reset(new tf2_ros::Buffer());
		tfl_.reset(new tf2_ros::TransformListener(*tf_));

		particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
	}
	~MclNode()
	{
		delete pf_;
	}

	void loop(void)
	{
		double x, y, t;
		if(not getOdomPose(latest_odom_pose_, x, y, t)){
			ROS_INFO("can't get odometry info");
			return;
		}

		pf_->updateOdom(x, y, t);

		publishParticles();

		sendTf();
	}

private:
	ParticleFilter *pf_;
	ros::NodeHandle nh_;

	ros::Publisher particlecloud_pub_;

	string base_frame_id_;
	string global_frame_id_;
	string odom_frame_id_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
	std::shared_ptr<tf2_ros::TransformListener> tfl_;
	std::shared_ptr<tf2_ros::Buffer> tf_;

	geometry_msgs::PoseStamped latest_odom_pose_;

	void publishParticles(void)
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

	void sendTf(void)
	{
		geometry_msgs::TransformStamped tmp_tf_stamped;
		tmp_tf_stamped.header.frame_id = global_frame_id_;
		tmp_tf_stamped.child_frame_id = odom_frame_id_;
		tmp_tf_stamped.header.stamp = ros::Time::now();

		tmp_tf_stamped.transform.translation.x = 0.0;
		tmp_tf_stamped.transform.translation.y = 0.0;
		tmp_tf_stamped.transform.translation.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, 0);
		tmp_tf_stamped.transform.rotation.x = q.x();
		tmp_tf_stamped.transform.rotation.y = q.y();
		tmp_tf_stamped.transform.rotation.z = q.z();
		tmp_tf_stamped.transform.rotation.w = q.w();

		this->tfb_->sendTransform(tmp_tf_stamped);
	}

	bool getOdomPose(geometry_msgs::PoseStamped& odom_pose, double& x, double& y, double& yaw)
	{
		geometry_msgs::PoseStamped ident;
		ident.header.frame_id = base_frame_id_;
		ident.header.stamp = ros::Time(0);
		tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
		
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

};

int main(int argc, char **argv)
{

	ros::init(argc,argv,"mcl_node");
	MclNode node;

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ROS_INFO("%s", "send");

		node.loop();

		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();

	return 0;
}
