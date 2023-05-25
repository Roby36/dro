
#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

// Common C/C++ inclusions (to be placed in shared file between modules)
#include <cstdlib>

template <typename T> class PubHandler
{
    //** Publisher info **//
    ros::Publisher    pub;
    const std::string topicName;
    const int         queueSize;

    public:

    //** Constructor **//
    PubHandler( ros::NodeHandle* nh, const std::string topicName, 
                    const int queueSize = 10);
    
    //** Publish custom message built outside **//
    void publish(const T& msg);
};
