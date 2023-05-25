
/* This module processes input and output messages at a lower level
 * without accessing publishers / subscribers / services
 */
#pragma once
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "../transforms/TransformBroadcaster2.h"

class VelController
{
    //** Transform used to switch between frames and listener **//
    tf::StampedTransform T; 
    tf::TransformListener listener;

    //** Working frame for the controller (usually base_link_orientation) **//
    const std::string working_frame_id;

    /* Helper function to find array indices in LaserMsg array
    *  given corresponding angular range 
    */
    static inline void ranges_indices(const sensor_msgs::LaserScan& laser_msg,
                                      const double min_scan_angle,
                                      const double max_scan_angle,
                                              int& min_index,
                                              int& max_index);
    
    bool update_transform(const std::string& target_frame, 
                          const std::string& source_frame);
 
    public:

    //** Constructor **//
    VelController(const std::string working_frame_id = "base_link_orientation")
        : working_frame_id(working_frame_id)
    {
    }

    /** Algorithm for general 360° obstacle detection
    /* 
    *   INPUTS:
    *   - lv, av: 
    *       input velocity's linear and angular components
    *   - input_vel_frame_id: 
    *       frame of input velocity commands
    *   - output_vel_frame_id:
    *       frame of output velocity command
    *   - laser_msg:
    *       message coming from laser sensor (already includes frame)
    *   - input_laser_frame_id:
    *       frame of input laser message
    *   - ang_range:
    *       angular range over which to perform obstacle scan
    *   - thresh_distance
    *       threshold distance for obstacle detection
    *   
    *   OUTPUTS:
    *   - cmd_vel_msg:
    *       output velocity message (reference)
    *   - success:
    *       boolean indicating whether command executed or ignored
    * 
    *   NOTES:
    *   - av currently ignored
    *   - frame_ids can be extracted from message stamps?
    *  
    */
    bool omnidirectional_obstacle_check(const tf::Vector3& lv, 
                                        const tf::Vector3& av, 
                                        const std::string& input_vel_frame_id,
                                        const std::string& output_vel_frame_id,
                                        const std::string& input_laser_frame_id,
                                        const double ang_range,
                                        const double thresh_distance,
                                        const sensor_msgs::LaserScan& laser_msg,
                                              geometry_msgs::TwistStamped& cmd_vel_msg);

    /* COMING NEXT:
    * - Upward obstacle detection with additional rangefinder sensor
    * - Downward obstacle detection
    * - PID controller
    */
    
};
