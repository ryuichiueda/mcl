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
	//	ros::Subscriber sub = nh_.subscribe("/cmd_vel", 1, &MclNode::receive_cmdvel, this);
	}
	~MclNode()
	{
		delete pf_;
	}

	/*
	void receive_cmdvel(const geometry_msgs::Twist::ConstPtr& msg)
	{
		pf_->setVelocity(msg->linear.x, msg->angular.z);
		ROS_INFO("linear: %f", msg->linear.x);
		ROS_INFO("angular: %f", msg->angular.z);
	}
	*/

private:
	Pf *pf_;
	ros::NodeHandle nh_;

	string base_frame_id_;
	string global_frame_id_;
	string odom_frame_id_;
};

int main(int argc, char **argv)
{

	ros::init(argc,argv,"mcl_node");

	/*
	while(!ros::service::waitForService("/static_map", ros::Duration(3.0))){
		ROS_INFO("Waiting for static_map");
	}

	ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("/static_map");

	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response res;
	if(not client.call(req, res)){
		ROS_ERROR("static_map not working");
		return 1;
	}
	*/

	MclNode mcl();

	/*
	XmlRpc::XmlRpcValue vi_node;
	n.getParam("/vi_node", vi_node);
	ROS_ASSERT(vi_node.getType() == XmlRpc::XmlRpcValue::TypeStruct);

	ValueIterator value_iterator(res.map, vi_node);
	ViActionServer vi_server(n, value_iterator);
	*/

	ros::spin();

	return 0;
}
