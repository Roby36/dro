
#pragma once

#include <ros/ros.h> 
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <string> 
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <cstdint>  // for integer types
#include <curses.h> // keyboard input

const int frequency  = 1000; // while-loop frequency
const int tout       = 100;  // ncurses timeout                                 
const int queue_size = 10;

enum Keypress {
    UP        = 'e',
    DOWN      = 'x',
    CW        = 'd',
    CCW       = 's',
    FORWARDS  = 'i',
    BACKWARDS = 'm',
    RIGHT     = 'k',
    LEFT      = 'j', 
    QUIT      = 'q'
};

class JController
{
    /* PUBLISHERS */
    ros::Publisher cmd_vel_pub;
 
    /* TRANSFORMER & TRANSFORMS */
    tf::TransformListener listener;
    tf::StampedTransform  blo_to_odom_transform; 

    /* DRONE DEFAULT LINEAR AND ANGULAR VELOCITIES */
    const double linearX;
    const double linearY;
    const double linearZ;
    const double angularX;
    const double angularY;
    const double angularZ;  

    /* Method to publish linear and angular velocities */
    void publish_velocities(double, double, double,
                            double, double, double);

    public:

    /* Constructor */
    JController(ros::NodeHandle*,
        double, double, double, 
        double, double, double);
    
    /* Methods to handle input */
    bool handleCommand(char);
    void handleKeypress();

};
