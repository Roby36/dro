
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

#include "../handlers/PubHandler.h"
#include "../handlers/SubHandler.h"
#include "PID.h"

class VelController
{
    /** Velocity publisher handler (initialized outside) **/
    PubHandler <geometry_msgs::TwistStamped> * const vel_ph;

    /** Laser subscriber handler (initialized outside) **/
    SubHandler <sensor_msgs::LaserScan> * const laser_sh;

    /** PID (initialized at default values) **/
    PID* const pid_ctr;

    //** Transform used to switch between frames and listener **//
    tf::StampedTransform T; 
    tf::TransformListener listener;

    //** Frames for the controller **//
    const std::string working_frame_id;
    const std::string output_vel_frame_id;
    const std::string input_laser_frame_id;

    /* Helper function to find array indices in LaserMsg array
     *  given corresponding angular range 
     */
    static inline void ranges_indices(const sensor_msgs::LaserScan& laser_msg,
                                      const double min_scan_angle,
                                      const double max_scan_angle,
                                              int& min_index,
                                              int& max_index);

    //** Check if laser message reading is within range **//
    static bool is_within_range(const sensor_msgs::LaserScan& laser_msg, const int i);
    
    //** Computing a minimum distance given a laser scan message **//
    static double min_distance(const sensor_msgs::LaserScan& msg,
                               const double min_scan_angle, 
                               const double max_scan_angle);

    
    //** Lookup transform handling exception **//
    bool update_transform(const std::string& target_frame, 
                          const std::string& source_frame);

    public:

    //** Constructor **//
    VelController(PubHandler <geometry_msgs::TwistStamped> * const vel_ph,
                  SubHandler <sensor_msgs::LaserScan>      * const laser_sh,
                  PID* pid_ctr,
                  const std::string working_frame_id     = "base_link_orientation",
                  const std::string output_vel_frame_id  = "odom",
                  const std::string input_laser_frame_id = "base_scan")
                    : working_frame_id(working_frame_id),
                      output_vel_frame_id(output_vel_frame_id),
                      input_laser_frame_id(input_laser_frame_id),
                      vel_ph(vel_ph), laser_sh(laser_sh), pid_ctr(pid_ctr)
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
    *   - this function does NOT set the timestamp of the message 
    *   - av currently ignored
    *   - frame_ids can be extracted from message stamps?
    *  
    */
    bool omnidirectional_obstacle_check(const tf::Vector3& lv, 
                                        const tf::Vector3& av, 
                                        const std::string& input_vel_frame_id,
                                        const double ang_range,
                                        const double thresh_distance,
                                        const sensor_msgs::LaserScan& laser_msg,
                                              geometry_msgs::TwistStamped& cmd_vel_msg);

    /* COMING NEXT:
    * - Upward obstacle detection with additional rangefinder sensor
    * - Downward obstacle detection
    * - PID controller
    */

   //** Reset internal PID controller **//
   void reset_PID(const double K,
                  const double Kp,
                  const double Ki,
                  const double Kd,
                  const int pid_error_sum_terms = -1);

   //** Algortithm to follow wall with PID controller (similar to PA2) **//
   void follow_wall(const double obst_thresh_distance,
                    const double obst_scan_angle,
                    const double obst_ang_range,
                    const double wall_goal_distance,
                    const double wall_scan_angle,
                    const double wall_ang_range,
                    const double linear_velocity,
                    const double angular_velocity,
                    const double pid_dt, // PID parameters:
                    const int    loop_frequency);
    /* COMING NEXT:
    * - Possibility to follow fall on left / right
    *
    */

    
};
