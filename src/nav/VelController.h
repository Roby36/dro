
/* This module processes input and output messages at a lower level
 * without accessing publishers / subscribers / services
 */
#pragma once
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "../transforms/TransformBroadcaster2.h"
#include "../handlers/PubHandler.h"
#include "../handlers/SubHandler.h"
#include "PID.h"

//! Tested & tuned PID parameters for obstacle circumvention
namespace PID_settings
{
    // PID parameters
    const double K  = 0.0;
    const double Kp = 0.3;
    const double Ki = 0.0;
    const double Kd = 0.2;
    const double err_sum_terms = -1;
    // follow_wall() function, using PID
    const double obst_thresh_distance = 3.0;
    const double obst_scan_angle      = 0.0;
    const double obst_ang_range       = M_PI/8.0;
    const double wall_goal_distance   = 3.0;
    const double wall_scan_angle      = -M_PI/2.0;
    const double wall_ang_range       = 3.0; // just below PI to avoid discontinuities
    const double linear_velocity      = 0.5;
    const double angular_velocity     = 0.5;
    const double dt                   = 0.1;
    const int    loop_frequency       = 100;
};
//!

class VelController
{
    /** Logger (still to be implemented) **/
    const std::string logfile;

    /** Velocity publisher handler (initialized outside) **/
    PubHandler <geometry_msgs::TwistStamped> * const vel_ph;

    /** Laser & Odometry subscriber handlers (initialized outside) **/
    SubHandler <sensor_msgs::LaserScan> * const laser_sh;
    SubHandler <nav_msgs::Odometry>     * const odom_sh;

    /** PID (initialized at default values above) **/
    PID* const pid_ctr;

    //** Transform used to switch between frames and listener **//
    tf::StampedTransform T; 
    tf::TransformListener listener;

    //** Frames for the controller **//
    const std::string working_frame_id;
    const std::string output_vel_frame_id;
    const std::string input_laser_frame_id;

    public:

    //** Constructor **//
    VelController(PubHandler <geometry_msgs::TwistStamped> * const vel_ph,
                  SubHandler <sensor_msgs::LaserScan>      * const laser_sh,
                  SubHandler <nav_msgs::Odometry>          * const odom_sh,
                  PID* pid_ctr,
                  const std::string working_frame_id     = "base_link_orientation",
                  const std::string output_vel_frame_id  = "odom",
                  const std::string input_laser_frame_id = "base_scan",
                  const std::string logfile              = "VelController")
                    : working_frame_id(working_frame_id),
                      output_vel_frame_id(output_vel_frame_id),
                      input_laser_frame_id(input_laser_frame_id),
                      vel_ph(vel_ph), laser_sh(laser_sh), odom_sh(odom_sh), 
                      pid_ctr(pid_ctr), logfile(logfile)
    {
    }

    /** ALGORITHM: omnidirectional_obstacle_check 
    *
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
    *   - Upward obstacle detection with additional rangefinder sensor
    *   - Downward obstacle detection   
    */
    bool omnidirectional_obstacle_check(const tf::Vector3& lv, 
                                        const tf::Vector3& av, 
                                        const std::string& input_vel_frame_id,
                                        const double ang_range,
                                        const double thresh_distance,
                                        const sensor_msgs::LaserScan& laser_msg,
                                              geometry_msgs::TwistStamped& cmd_vel_msg);

   /** ALGORITHM: follow_wall()
    * 
    *  NOTES:
    *   This algorithm functions like a differential drive robot,
    *   using 2 degrees of freedom. It takes wall distance as PID input
    *   and angular velocity as PID output. Despite it can be tuned to 
    *   function at a satisfactory level, we still want to enhance it to
    *   exploit all the additional degrees of freedom of the drone.
    *   This will be important since Bug2 & Bug3 are built directly on top
    *   of this algorithm.
    */
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
   
    /** ALGORITHM: Bug2 
    *   This algorithm uses the two algorithms above, and parameters in 
    *   PIDsettings, to implement the Bug2 algorithm seen in class 
    */
    bool Bug2( const std::string& goal_frame_id,
               const tf::Vector3& goal_point,
               const tf::Vector3& goal_tol,
               const tf::Vector3& linear_velocity,
               const tf::Vector3& angular_velocity,
               const tf::Vector3& ang_tol,
               const tf::Vector3& line_tol,
               const double       obst_ang_range,
               const double       obst_thresh_distance,
               const int          frequency);
    
    //** Reset internal PID controller **//
   void reset_PID(const double K,
                  const double Kp,
                  const double Ki,
                  const double Kd,
                  const int pid_error_sum_terms = -1);
    

    //** Private helper-function used internally by the module **//
    private:

   //** Message-processing functions (from subscribers) **//
    static inline void ranges_indices(const sensor_msgs::LaserScan& laser_msg,
                                      const double min_scan_angle,
                                      const double max_scan_angle,
                                      int& min_index,
                                      int& max_index);
    static bool is_within_range(const sensor_msgs::LaserScan& laser_msg, const int i);
    static double min_distance(const sensor_msgs::LaserScan& msg,
                               const double min_scan_angle, 
                               const double max_scan_angle);
    static void getRPY(const nav_msgs::Odometry& odom_msg, 
                       double& roll,
                       double& pitch,
                       double& yaw);
    //** Transforms **//
    bool update_transform(const std::string& target_frame, 
                          const std::string& source_frame);
    //** Geometrical functions **//
    static inline double absolute_value(double x);
    static inline bool is_close( const tf::Vector3& v1,
                                 const tf::Vector3& v2,
                                 const tf::Vector3& tol);
    static bool intersects_line( const tf::Vector3& start,
                                 const tf::Vector3& goal,
                                 const tf::Vector3& curr_pos,
                                 const tf::Vector3& tol);
    void inline update_position_vector( tf::Vector3& v);
    static inline double xy_distance( tf::Vector3& v1, tf::Vector3& v2);
    //** Motion functions (publishing) **//
    void stop();
    void rotate( const tf::Vector3& targetOr, 
                 const tf::Vector3& tol, 
                 const tf::Vector3& ang_vel,  
                 int frequency);
    
};
