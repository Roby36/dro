
#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

// Common C/C++ inclusions (to be placed in shared file between modules)
#include <cstdlib>

template <typename T> class SubHandler
{
    //** Currently retrieved message **//
    T msg;

    //** Flag variable signalling message receival **//
    bool initialized = false;

    //** Subscriber info **//
    ros::Subscriber   sub;
    const std::string topicName;
    const int         queueSize;

    //** Subscriber callback function **//
    void callback(const T& msg);

    public:

    //** Constructor **// 
    SubHandler( ros::NodeHandle* nh, const std::string topicName, 
        const int queueSize = 10); // queueSize defaults to 10

    //** Getters **// 
    T const currMsg() { return msg;}
    bool const is_initialized() { return initialized;}
};