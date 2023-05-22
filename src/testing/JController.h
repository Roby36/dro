
#pragma once

#include <ros/ros.h> 
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "../handlers/ServiceHandler.h"
#include "../handlers/VelPubHandler.h"
#include "../handlers/SubHandler.h"

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

//** Keyboard commands **//
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
    /* Velocity handler (initialized elsewhere) */
    VelPubHandler * const vel_ph;

    /* Scan width and depth */
    const double min_scan_angle;
    const double max_scan_angle;
    const double min_distance;

    /* Drone default linear and angular velocities */
    tf::Vector3 const lv;
    tf::Vector3 const av;  

    public:

    /* Constructor */
    JController(VelPubHandler * const vel_ph,
                const tf::Vector3&    lv,
                const tf::Vector3&    av,
                const double          min_scan_angle =  1.0, 
                const double          max_scan_angle = -1.0,
                const double          min_distance   = 6.0)
        : vel_ph(vel_ph), lv(lv), av(av),
          min_scan_angle(min_scan_angle), max_scan_angle(max_scan_angle),
          min_distance(min_distance)
    {
    }

    /* Methods to handle input */
    bool handleCommand(char);
    void handleKeypress();

};
