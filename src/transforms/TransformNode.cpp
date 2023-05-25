
#include "TransformBroadcaster2.h"

//** Constants **//
const std::string nodeName  = "TransformNode";
const std::string odomTopic = "/mavros/local_position/odom";

int main(int argc, char** argv) 
{
    // Initialize node
    ros::init(argc, argv, ::nodeName);
    ros::NodeHandle nh;
    // Initialize odom subscription handler
    SubHandler <nav_msgs::Odometry> odom_sh (&nh, ::odomTopic);
    // Construct Broadcaster
    TransformBroadcaster bc (&odom_sh);
    // Run broadcaster on this thread until node termination
    bc.run();
    // Clean up
    return 0;
}