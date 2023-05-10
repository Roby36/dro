
#pragma once

#include <ros/ros.h> 
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
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
const int tout    = 100;    // ncurses timeout                                 
const int queue_size = 10;

enum BaseMode : uint8_t {
    MAV_MODE_PREFLIGHT          = 0,
    MAV_MODE_STABILIZE_DISARMED = 80,
    MAV_MODE_STABILIZE_ARMED    = 208,
    MAV_MODE_MANUAL_DISARMED    = 64,
    MAV_MODE_MANUAL_ARMED       = 192,
    MAV_MODE_GUIDED_DISARMED    = 88,
    MAV_MODE_GUIDED_ARMED       = 216,
    MAV_MODE_AUTO_DISARMED      = 92,
    MAV_MODE_AUTO_ARMED         = 220,
    MAV_MODE_TEST_DISARMED      = 66,
    MAV_MODE_TEST_ARMED         = 194
};

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

    /* SERVICE CLIENTS */
    ros::ServiceClient mode_srv;
    ros::ServiceClient arming_srv;
    ros::ServiceClient takeoff_srv;
    ros::ServiceClient land_srv;

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
    
    /* SERVICE REQUESTS METHODS */
    bool reqMode(BaseMode, std::string);
    bool reqArming(bool);
    bool reqTakeoff(float, float, float, float, float);
    bool reqLand(float, float, float, float, float);

    /* Method to handle input */
    bool handleCommand(char);
    void handleKeypress();

};
