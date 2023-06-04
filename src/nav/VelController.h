
#pragma once
// C inclusions
#include <cmath>
#include <math.h>
#include <stdio.h>
// ROS inclusions
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
// Other modules in the project
#include "../transforms/TransformBroadcaster2.h"
#include "../handlers/PubHandler.h"
#include "../handlers/SubHandler.h"
#include "logger.h"
#include "PID.h"

/** Struct defining general parameters for a 2-D laser scan **/ 
typedef struct SP {

  double ang_midpoint;
  double ang_range;
  double distance;

  SP( double ang_midpoint, double ang_range, double distance)
    : ang_midpoint(ang_midpoint), ang_range(ang_range), distance(distance) {}

} ScanParameters;

class VelController
{
  //** General navigation parameters **//
  const double         linear_speed;
  const tf::Vector3    point_tol; // tolerance used when reaching points
  const double         angular_velocity; // we will only use the z (yaw) rotational dof
  const double         ang_tol; // tolerance used when rotating
  ScanParameters osp;          // obstacles
  ScanParameters wsp;         // walls
  const int            loop_frequency; // frequency at which cmd_vel messages published

  /** Logging **/
  Logger m_logger;

  /** Velocity publisher handler (initialized outside) **/
  PubHandler <geometry_msgs::TwistStamped> * const vel_ph;

  /** Laser, Odometry, rangefinder subscriber handlers (initialized outside) **/
  SubHandler <sensor_msgs::LaserScan> * const laser_sh;
  SubHandler <nav_msgs::Odometry>     * const odom_sh;
  SubHandler <sensor_msgs::Range>     * const range_sh;

  /** PID controllers **/
  PID * const obst_pid; 
  PID * const  alt_pid;  

  //** Transform used to switch between frames and listener **//
  tf::StampedTransform T; 
  tf::TransformListener listener;

  //** Frames for the controller **//
  const std::string working_frame_id;
  const std::string output_vel_frame_id;
  const std::string input_laser_frame_id;

  /** Methods are all public for the sake of easier testing **/
  public:

  //** Constructor **//
  VelController(PubHandler <geometry_msgs::TwistStamped> * const vel_ph,
                SubHandler <sensor_msgs::LaserScan>      * const laser_sh,
                SubHandler <nav_msgs::Odometry>          * const odom_sh,
                SubHandler <sensor_msgs::Range>          * const range_sh,
              /* PID controllers */
                PID *  obst_pid,
                PID *  alt_pid,
              /* Navigation parameters */
                const double         linear_speed,
                const tf::Vector3    point_tol,
                const double         angular_velocity,
                const double         ang_tol,
                const ScanParameters osp,
                const ScanParameters wsp,
                const int            loop_frequency,
              /* Frames */
                const std::string working_frame_id     = "base_link_orientation",
                const std::string output_vel_frame_id  = "odom",
                const std::string input_laser_frame_id = "base_scan",
                const std::string logpath              = "/root/catkin_ws/ardupilot_ws/src/dro/logs/VC_log.txt");
  
  //** Destructor **//
  ~VelController();

  /** omnidirectional_obstacle_check
  * 
  * This function acts as a "filter" to detect obstacles in a given direction of motion.
  * It takes as paramters vectors for the requested velocity in a given frame, 
  * the most updated laser scan message, and the velocity message to fill in.
  * Based on the sensor data, the function fills up the velocity message appropriately
  * to avoid potential obstacles. The flag parameter unchecked can be used to fill up the 
  * velocity message regardless of the potential obstacles in the drone's way
  */
  bool omnidirectional_obstacle_check(const tf::Vector3& lv, 
                                      const tf::Vector3& av, 
                                      const std::string& input_vel_frame_id,
                                      const sensor_msgs::LaserScan& laser_msg,
                                      geometry_msgs::TwistStamped& cmd_vel_msg,
                                      bool unchecked = false);
  /**  follow_wall
  *
  * This function is used to perform PID iterations when the drone circumvents an obstacle.
  * 
  * PARAMETERS:
  *  - dt: time period in seconds for the PID iteration
  *  - orbit_angle: required angle between the drone's velocity vector
  *       and the drone's yaw throughout the drone's circumvention of the obstacle.
  *       For instance, if orbit_angle = 0, the drone goes around the obstacle facing 
  *       directly towards the direction of motion, similarly to a differential drive robot
  *  - climb_angle: angle between the horizontal plane and the drone's velocity vector, 
  *       determining the rate of altitude gain as the drone goes around the obstacle
  *  - wall_contact, close_obstacle: flag variables signalling a problem as the drone iterates
  *       around the obstacle. These variables are used to determine when to call the appropriate
  *       handlers to correct the drone's path.
  */
  bool follow_wall( const double dt, 
                /* Angle between velocity vector and drone's yaw */
                    const double orbit_angle,
                /* Rate of altitude gain */
                    const double climb_angle,
                /* Optional flag(s), signalling problems */
                    bool& wall_contact,
                    bool& close_obstacle);
  /** Bug3
  * This is the main algorithm of the module, guiding the drone to an arbitrary point 
  * in an arbitrary frame whilst avoiding the obstacle in the way  
  * 
  * PARAMETERS:
  *  - goal_frame_id: the frame in which the goal point is expreseed
  *  - goal_point: the point the drone needs to reach in the given frame
  *  - min_agl, max_agl: the minimum and maximum altitudes above ground level (AGL)
  *       allowed as the drone travels towards the goal point
  *  - climb_ang_policy: this function maps the current AGL of the drone to the climb angle 
  *       when the drone is in obstacle avoidance mode. It works similarly to a PID controller.
  *       For instance, if the drone flies above maximum AGL, the function will output a negative 
  *       climb angle to account for that.
  *  - obst_pid_params: The PID parameters used in obstacle avoidance mode. The input is the 
  *       minimum distance between the drone and wall, and the output is the angle of the drone's
  *       velocity with respect to the wall. follow_wall() will use SHM motion equations to ensure the
  *       angle between the drone's yaw and velocity remains constant throghout the obstacle 
  *       circumvention, so that the drone can keep a constant orbit_angle
  *  - alt_pid_params: PID parameters used by the altitude-regulating PID controller of the drone
  *  - dt, orbit_angle: These are the parameters passed to follow_wall() when the drone goes around an obstacle
  *  - side_vel: This is the velocity used by the drone when it comes dangerously close to an obstacle
  *       due to a PID failure     
  */
  bool Bug3(const std::string& goal_frame_id,
            const tf::Vector3& goal_point, 
      /* Altitude ABOVE GROUND bounds */
            const double       min_agl,
            const double       max_agl,
      /* Function determining climb angle when travelling around obstacle */
            double (*climb_ang_policy) (double, double, double),    
      /* Parameters for PID */
            const PIDparams& obst_pid_params,
            const PIDparams& alt_pid_params,
      /* Parameters for follow_wall phase */
            const double dt, 
            const double side_vel,
            const double orbit_angle);

  /** bug3_obstacle_handler
  * 
  * Helper-function used by Bug3 when encountering an obstacle
  */
  void bug3_obstacle_handler(const tf::Vector3& start_odom,
                             const tf::Vector3& goal_odom, 
                    /* Altitude ABOVE GROUND bounds */
                             const double       min_agl,
                             const double       max_agl,
                    /* Function determining climb angle when travelling around obstacle */
                             double (*climb_ang_policy) (double, double, double),    
                    /* Parameters for PID */
                             const PIDparams& obst_pid_params,
                    /* Parameters for follow_wall phase */
                             const double dt, 
                             const double side_vel,
                             const double orbit_angle);
  /** vertical_vel_check
  *
  * Helper-function used by Bug3 to enforce altitude constraints.
  * This function uses a PID controller to maintain the drone's 
  * altitude within the desired range
  */
  void vertical_vel_check(bool& alt_within_range,
                          double& vertical_vel,
                          const double min_agl,
                          const double max_agl,
                          const PIDparams& alt_pid_params);
  
  /** A simple linear climbing angle policy:
  * This function returns a climb angle between [0, PI/2],
  * corresponding to an altitude value included between [min_agl, max_agl]
  */
  static double lin_ang_pol(double alt, double min_agl, double max_agl);
  
  /** Reset internal PID controller **/
  void reset_PID( const PIDparams nparams);
  
  /** ZN_tuning_test
  * 
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
                    /* Parameters for follow_wall() */
                      const double dt, 
                      const double side_vel,
                      const double orbit_angle,
                      const double climb_angle);
  /** test_wall_pid_params
  *  
  * Helper-function used by ZN_tuning_test to test each individual set of PID parameters
  */
  bool test_wall_pid_params(const PIDparams params,
                            const double time_goal,
                            const double dt, 
                            const double orbit_angle, 
                            const double climb_angle,
                      /* Information for handling PID failures */
                            bool& wall_contact,
                            bool& close_obstacle);
  /** rotate_in_line()
  * 
  * This function rotates the drone around itself whilst following a constant 
  * 3-D linear path with respect to a fixed global reference frame. It is used as 
  * a test for the SHM equations used in follow_wall() where the drone has to maintain
  * a constant orbiting angle whilst changing x,y linear velocity components and yaw.
  * 
  *  INPUTS:
  *   - linear velocity vector (in the fixed global frame) the drone has to follow
  *   - angular velocity at which we want the drone to rotate around itself,
  *     with respect to base_link
  *   - execution time, after which the function returns
  * 
  *  OUTUTS:
  *   - the function translates the commands in the base-link reference frame,
  *     publishes the relevant velocity messages, and returns after the time limit.
  *     The z-component of linear velocity will remain constant throughout execution
  */
  void rotate_in_line( const tf::Vector3& linear_vel,
                       const std::string& input_vel_frame_id,
                       const double ang_vel,
                       const double exec_time);

  /** Motion functions to move the drone */
  void rotateYaw( const double input_yaw, 
                  const double tol, 
                  const double ang_vel,  
                  int frequency);
  void stop();
  bool change_alt(double alt_gain, 
                  double climb_vel, 
                  double max_dur = 100.0);

  //** Functions handling PID failures **//
  double get_wall_tangential_angle(const tf::Vector3& obst_pos,
                                   const std::string& obst_frame_id);
  bool get_closest_obstacle_position(tf::Vector3& obst_pos,
                                     std::string& obst_frame_id);
  bool handle_lost_wall_contact(tf::Vector3& obst_pos,
                                std::string& obst_frame_id,
                                double dur = 10.0);
  bool handle_missed_wall(const tf::Vector3& obst_pos,
                          const std::string& obst_frame_id,
                        /* Drone translation after PID failure */
                          const double side_vel,
                          const double orbit_angle,
                          const bool final_rot_adjustment = false);
  
  //** Laser message processing functions (from subscribers) **//
  static inline void ranges_indices(const sensor_msgs::LaserScan& laser_msg,
                                    const double min_scan_angle,
                                    const double max_scan_angle,
                                    int& min_index,
                                    int& max_index);
  static inline void bring2range( const double angle_min,
                                  const double angle_max,
                                  double& angle);
  static inline bool is_within_range(const sensor_msgs::LaserScan& laser_msg, const int i);
  static inline bool is_within_range(const sensor_msgs::Range& range_msg);
  static double min_distance(const sensor_msgs::LaserScan& laser_msg,
                             double min_scan_angle, 
                             double max_scan_angle,
                          /* Optional parameter keeping angle of minimum distance */
                             double& min_dist_ang);
  double min_wall_distance( const sensor_msgs::LaserScan& laser_msg,
                            const ScanParameters& wsp);
  static void getRPY(const nav_msgs::Odometry& odom_msg, 
                      double& roll,
                      double& pitch,
                      double& yaw);
  
  //** Transforms **//
  bool update_transform(const std::string& target_frame, 
                        const std::string& source_frame);
  
  //** Geometrical computations functions **//
  static inline double absolute_value(double x);
  static inline bool is_close( const tf::Vector3& v1,
                               const tf::Vector3& v2,
                               const tf::Vector3& tol);
  static bool intersects_line( const tf::Vector3& start,
                               const tf::Vector3& goal,
                               const tf::Vector3& curr_pos,
                               const tf::Vector3& tol);
  void inline update_position_vector( tf::Vector3& v);
  static inline double xy_distance ( const tf::Vector3& v1, 
                                     const tf::Vector3& v2);
  static inline double xyz_distance( const tf::Vector3& v1, 
                                     const tf::Vector3& v2);
  
  //** Overloaded copies (to make reference parameters optional) **//
  bool follow_wall( const double dt,
                    const double orbit_angle,
                    const double climb_angle);
  static double min_distance(const sensor_msgs::LaserScan& laser_msg,
                             double min_scan_angle, 
                             double max_scan_angle);

};


