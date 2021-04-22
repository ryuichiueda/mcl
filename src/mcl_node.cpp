#include <ros/ros.h>
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
/*
#include <iostream>
#include <vector>
*/
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"mcl_node");
	ros::NodeHandle n;

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
