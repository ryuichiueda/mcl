# mcl

mcl is an alternative Monte Carlo localization (MCL) package to amcl (http://wiki.ros.org/amcl). Differently from amcl, KLD-samping and adaptive MCL are not implemented. 

This package is suitable for learning MCL due to its simplicity.


## Nodes

### mcl_node

This node transforms laser scans and odometry transform messages to pose estimations by using an occupancy grid map. 

#### Subscrbed Topics 

* scan ([sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html))
    * laser scans
* tf ([tf/tfMessage](http://docs.ros.org/en/api/tf/html/msg/tfMessage.html))
    * transforms

#### Published Topics

* mcl_pose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
    * the mean pose of the particles with covariance
* particlecloud ([geometry_msgs/PoseArray](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseArray.html))
    * poses of the particles
* tf ([tf/tfMessage](http://docs.ros.org/en/api/tf/html/msg/tfMessage.html))
    * the transform from odom (which can be remapped via the ~odom_frame_id parameter) to map


#### Services Called

* static_map ([nav_msgs/GetMap](http://docs.ros.org/en/api/nav_msgs/html/srv/GetMap.html))
    * mcl initializes the map for localization
