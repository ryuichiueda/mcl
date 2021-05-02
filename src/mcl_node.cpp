/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some parts are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include "mcl/interface.h"
using namespace std;

int main(int argc, char **argv)
{

	ros::init(argc, argv, "mcl_node");
	MclNode node;

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		node.loop();

		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();

	return 0;
}
