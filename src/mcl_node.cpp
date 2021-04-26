/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *  All rights reserved.
 *  Some parts are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include <ros/ros.h>
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Twist.h"
#include "Pf.h"
/*
#include <iostream>
#include <vector>
*/

//#include "tf2_ros/buffer.h"
//#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std;

class MclNode
{
public:
	MclNode()
	{
		pf_ = new Pf();
		global_frame_id_ = "map";
		base_frame_id_ = "base_link";
		odom_frame_id_ = "odom";

		tfb_.reset(new tf2_ros::TransformBroadcaster());
	}
	~MclNode()
	{
		delete pf_;
	}

	void loop(void)
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

private:
	Pf *pf_;
	ros::NodeHandle nh_;

	string base_frame_id_;
	string global_frame_id_;
	string odom_frame_id_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
	std::shared_ptr<tf2_ros::TransformListener> tfl_;
	std::shared_ptr<tf2_ros::Buffer> tf_;
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
