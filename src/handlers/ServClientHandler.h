
/* This module is a future idea still to be implemented,
 * to possibly avoid code repetition in ServiceHandler.cpp
 */

#pragma once

#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>

// Common C/C++ inclusions (to be placed in shared file between modules)
#include <cstdlib>

template <typename T> class ServClientHandler
{
    //** Service client **//
    ros::ServiceClient   serv_client;
    const std::string    servName;

    //** Request & Response **//

    public:

    //** Constructor **// 
    ServClientHandler(ros::NodeHandle* nh, const std::string servName); 

    //** Getters **// 
   
};