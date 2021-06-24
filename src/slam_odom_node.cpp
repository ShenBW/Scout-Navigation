#include <ros/ros.h>
#include "slam_odom.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_odom_publisher");
    ros::NodeHandle nh("~");

    slam_odom::SlamOdomPublisher odom_publisher(nh);

    odom_publisher.run();

    ros::spin();

    return 0;
}