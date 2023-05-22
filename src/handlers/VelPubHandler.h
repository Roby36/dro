
#pragma once

#include <ros/ros.h>

#include <cmath>
#include <math.h>

#include <ros/ros.h> 
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "SubHandler.h"

class VelPubHandler
{
    //** LaserScan subsriber handler **//
    SubHandler <sensor_msgs::LaserScan> * const laser_sh;

    //** Velocity publisher **//
    ros::Publisher cmd_vel_pub;

    /* TRANSFORMER & TRANSFORMS */
    tf::TransformListener listener;
    tf::StampedTransform  bloTodom; 

    public:

    //** Constructor **//
    VelPubHandler( ros::NodeHandle* nh,
                   SubHandler <sensor_msgs::LaserScan> * const laser_sh,
                   const std::string cmd_vel_topic_name,
                   const int queue_size = 10) : laser_sh(laser_sh)
    {
        // Initialize velocity publisher
        this->cmd_vel_pub = nh->advertise <geometry_msgs::TwistStamped> (cmd_vel_topic_name, queue_size); 
    }

    //** relative to base_link_orientation frame **//
    bool forward_obstacle_check(const tf::Vector3& lv, 
                                const tf::Vector3& av,
                                const double min_scan_angle = 0.0, 
                                const double max_scan_angle = 0.0,
                                const double min_distance   = 6.0);
    
};