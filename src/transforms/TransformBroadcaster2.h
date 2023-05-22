
#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include "../handlers/SubHandler.h"

class TransformBroadcaster
{
    //** Broadcaster **//
    tf::TransformBroadcaster br;

    //** Moving transforms **//
    tf::Transform odomTbl;  // odom -> "base link" transform
    tf::Transform odomTblo; // odom -> "base link orientation" transform

    //** Static transforms **//
    tf::Transform mapTodom; // map -> odom transform
    tf::Transform blTbs;    // "base_link" -> "base scan" transform

    //** Odometry subscription handler **//
    SubHandler<nav_msgs::Odometry> * const odom_sh;

    //** Broadcasting frequency **//
    const int frequency;

    //** Helper-function to broadcast each transform **//
    void broadcast_transform(const tf::Vector3&    position,
                             const tf::Quaternion& orientation,
                             const std::string&    parent_id, 
                             const std::string&    child_id,
                                   tf::Transform&  transform);

    //** Main broadcasting functions **//
    void broadcast_moving_frames();
    void broadcast_static_frames();

    public:

    //** Constructor **//
    TransformBroadcaster(SubHandler<nav_msgs::Odometry> * const odom_sh, 
        const int frequency = 10) : odom_sh(odom_sh), frequency(frequency)
    {
    }

    void run();
};