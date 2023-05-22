
#include "VelPubHandler.h"

bool VelPubHandler::forward_obstacle_check(const tf::Vector3& lv, 
                                           const tf::Vector3& av,
                                           const double min_scan_angle, 
                                           const double max_scan_angle,
                                           const double min_distance)
{
    // Prepare cmd_vel message
    geometry_msgs::TwistStamped vel_msg;
    // Update transform from base_link_orientation to odom,
    // since MAVROS interprets commands in odom frame
    try {
        listener.lookupTransform("odom", "base_link_orientation", ros::Time(0), this->bloTodom);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }
    // Use updated transform to set message parameters with respect to odom
    tf::Vector3 lt = this->bloTodom(lv);
    tf::Vector3 at = this->bloTodom(av);
    vel_msg.twist.linear.x  = lt.getX();
    vel_msg.twist.linear.y  = lt.getY();
    vel_msg.twist.linear.z  = lt.getZ();
    vel_msg.twist.angular.x = at.getX();
    vel_msg.twist.angular.y = at.getY();
    vel_msg.twist.angular.z = at.getZ();
    // Set message header and publish message
    vel_msg.header.frame_id = "odom";
    // Query Laser subscriber for most recent data
    do {
        ros::spinOnce();
    } while (!laser_sh->is_initialized());
    const sensor_msgs::LaserScan msg = laser_sh->currMsg();
    // Compute minimum and maximum index in ranges[] where these angles are located
    int min_index = std::max((int)floor((min_scan_angle - msg.angle_min) / msg.angle_increment) , 0);
    int max_index = std::min((int)ceil((max_scan_angle - msg.angle_min) / msg.angle_increment), 
        (int)((msg.angle_max - msg.angle_min) / msg.angle_increment));
    // Iterate through the given field of view to find obstacle:
    for (int i = min_index; i < max_index; i++) {
        if (msg.ranges[i] < min_distance &&
            msg.ranges[i] > msg.range_min &&
            msg.ranges[i] < msg.range_max &&
            // Block message only if drone attempts to move in positive 
            // X-direction wrt base_link reference frame
                //!! To be generalized for omnidirectional
                //!! obstacle avoidance ?
            lv.getX() > 0) {
            // If obstacle too close, return false without publishing velocities
            ROS_INFO_STREAM( "Stopping: obstacle detected within distance "
                            << min_distance << " at angle "
                            << i*msg.angle_increment + msg.angle_min);
            return false;
        }
    }
    // If no obstacle detected, stamp & publish desired velocities & return true
    vel_msg.header.stamp = ros::Time::now();
    this->cmd_vel_pub.publish(vel_msg);
    return true;
}