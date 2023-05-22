
#pragma once

#include <ros/ros.h> 
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>


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

class ServiceHandler {
    
    //** SERVICES **//
    ros::ServiceClient mode_srv;
    ros::ServiceClient arming_srv;
    ros::ServiceClient takeoff_srv;
    ros::ServiceClient land_srv;

    public:

    /* Constructor */
    ServiceHandler(ros::NodeHandle*);

    /* SERVICE REQUESTS METHODS */
    bool reqMode(BaseMode base_mode = MAV_MODE_PREFLIGHT, 
                 std::string custom_mode = "");
    bool reqArming(bool value);
    bool reqTakeoff(float min_pitch, float yaw, float latitude, 
                    float longitude, float altitude);
    bool reqLand(float min_pitch, float yaw, float latitude, 
                 float longitude, float altitude);

};