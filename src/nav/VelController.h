
#pragma once
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "../transforms/TransformBroadcaster2.h"
#include "../handlers/PubHandler.h"
#include "../handlers/SubHandler.h"
#include "logger.h"
#include "PID.h"

// Struct defining general parameters for a 2-D laser scan
typedef struct SP {

  double ang_midpoint;
  double ang_range;
  double distance;

  SP( double ang_midpoint, double ang_range, double distance)
    : ang_midpoint(ang_midpoint), ang_range(ang_range), distance(distance) {}

} ScanParameters;

class VelController
{
  public:

  //** Constructor **//
  VelController(PubHandler <geometry_msgs::TwistStamped> * const vel_ph,
                SubHandler <sensor_msgs::LaserScan>      * const laser_sh,
                SubHandler <nav_msgs::Odometry>          * const odom_sh,
                PID* pid_ctr,
                const std::string working_frame_id     = "base_link_orientation",
                const std::string output_vel_frame_id  = "odom",
                const std::string input_laser_frame_id = "base_scan",
                const std::string logpath              = "/root/catkin_ws/ardupilot_ws/src/dro/logs/VC_log.txt");

  //** Destructor **//
  ~VelController();

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
  void follow_wall( const ScanParameters& osp,
                    const ScanParameters& wsp,
                    const tf::Vector3& lv,
                    const tf::Vector3& av,
                    const double dt, 
                    const int    loop_frequency,
                  //! Optional parameter to assess pid success/ failure
                    bool& pid_success);
  
  //** Overloaded version of follow_wall to make last parameter optional **//
  void follow_wall( const ScanParameters& osp,
                    const ScanParameters& wsp,
                    const tf::Vector3& lv,
                    const tf::Vector3& av,
                    const double dt, 
                    const int    loop_frequency);
  
  /** ALGORITHM: Bug2 
  *   This algorithm uses the two algorithms above, and parameters in 
  *   PIDsettings, to implement the Bug2 algorithm seen in class 
  */
  bool Bug2(  const std::string& goal_frame_id,
              const tf::Vector3& goal_point,     
              const tf::Vector3& goal_tol,
              const tf::Vector3& linear_velocity,
              const tf::Vector3& angular_velocity,
              const tf::Vector3& ang_tol,
              const tf::Vector3& line_tol,
            //! Parameters for PID
              const PIDparams& pid_params,
            //! Parameters for follow_wall()
              const ScanParameters& osp,
              const ScanParameters& wsp,
              const tf::Vector3& lv,
              const tf::Vector3& av,
              const double dt, 
              const int    loop_frequency,
            //!
              const double       obst_ang_range,
              const double       obst_thresh_distance,
              const int          frequency);
  
  //** Reset internal PID controller **//
  void reset_PID( const PIDparams nparams);
  
  /** ZN_tuning_test
  * This function performs the Ziegler Nichols tuning process dynamically.
  * Proportional terms in a given range are tested and the values able to
  * oscillate sustainably for a given time are recorded. The minimum value 
  * able to constantly oscillate can then be used to tune the PID controller.
  */
  bool ZN_tuning_test(std::string  outfile,
                      const double Kp_start,
                      const double Kp_end,
                      const double Kp_step,
                      const double time_goal,
                      const tf::Vector3& obst_pos,
                      const std::string& obst_frame_id,
                      //! Parameters for follow_wall()
                      const ScanParameters& osp,
                      const ScanParameters& wsp,
                      const tf::Vector3& lv,
                      const tf::Vector3& av,
                      const double dt, 
                      const int    loop_frequency,
                      //! Parameters for rotating drone after PID failure
                      const tf::Vector3& ang_tol, 
                      const tf::Vector3& ang_vel, 
                      const int          rot_frequency,
                      //! Parameters for drone translation after PID failure
                      const double lin_vel,
                      const double wd_tol,
                      const int freq);

  //** private attributes/functions **//
  private:

  /** Logging **/
  Logger m_logger;

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

  //** Private helper-function used internally by the module **//

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
  double min_wall_distance( const sensor_msgs::LaserScan& laser_msg,
                            const ScanParameters& wsp);
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
