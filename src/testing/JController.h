
#pragma once

#include <ros/ros.h> 
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "./ServiceHandler.h"
#include "../nav/VelController.h"
#include "../nav/PID.h"
#include "../handlers/SubHandler.h"
#include "../handlers/PubHandler.h"

#include <string> 
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <cstdint>  // for integer types
#include <curses.h> // keyboard input                               

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

    NAVTEST   = 'T', 
    BUG2TEST  = 'B',
    PIDTEST   = 'P',
    ZNTEST    = 'Z',

    QUIT      = 'q'
};

class JController
{
    /** Ncurses / input variables **/
    const int frequency  = 1000; // while-loop frequency
    const int tout       = 100;  // ncurses timeout  

    /** Velocity publisher handler (initialized outside) **/
    PubHandler <geometry_msgs::TwistStamped> * const vel_ph;

    /** Laser subscriber handler (initialized outside) **/
    SubHandler <sensor_msgs::LaserScan> * const laser_sh;

    /** Velocity controller (initialized outside) **/
    VelController * const vel_ctr;

    /* VelController parameters */
    const double ang_range;
    const double thresh_distance;

    //** Velocity input frame **//
    const std::string input_vel_frame_id = "base_link_orientation";
  
    /* Set linear and angular velocities */
    tf::Vector3 const lv;
    tf::Vector3 const av;  

    /* Publishing velocities through obstacle check */
    void publish_checked_velocities(tf::Vector3 l_vel, tf::Vector3 a_vel);

    public:

    /* Constructor */
    JController(PubHandler <geometry_msgs::TwistStamped> * const vel_ph,
                SubHandler <sensor_msgs::LaserScan>      * const laser_sh,
                VelController * const vel_ctr,
                const tf::Vector3&    lv,
                const tf::Vector3&    av,
                const double ang_range       = M_PI / 2.0,
                const double thresh_distance = 3.0 )
        : vel_ph(vel_ph), laser_sh(laser_sh), vel_ctr(vel_ctr), lv(lv), av(av),
          ang_range(ang_range), thresh_distance(thresh_distance)
    {
    }

    /* Methods to handle input */
    bool handleCommand(char);
    void handleKeypress();

    /* Navigation tests (hard-coded) */
    void navtest();
    void bug2test();
    void ZNtest();

};
