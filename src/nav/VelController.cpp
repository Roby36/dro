
#include "VelController.h"

//** Contructor & destructor **//
VelController::VelController(PubHandler <geometry_msgs::TwistStamped> * const vel_ph,
                            SubHandler <sensor_msgs::LaserScan>      * const laser_sh,
                            SubHandler <nav_msgs::Odometry>          * const odom_sh,
                            PID* pid_ctr,
                            //! Navigation parameters
                            const tf::Vector3    linear_velocity,
                            const tf::Vector3    point_tol,
                            const double         angular_velocity,
                            const double         ang_tol,
                            const ScanParameters osp,
                            const ScanParameters wsp,
                            const int            loop_frequency,
                            //!
                            const std::string working_frame_id,
                            const std::string output_vel_frame_id,
                            const std::string input_laser_frame_id,
                            const std::string logpath)
                                :   working_frame_id(working_frame_id),
                                    output_vel_frame_id(output_vel_frame_id),
                                    input_laser_frame_id(input_laser_frame_id),
                                    linear_velocity(linear_velocity),
                                    point_tol(point_tol),
                                    angular_velocity(angular_velocity),
                                    ang_tol(ang_tol),
                                    osp(osp),
                                    wsp(wsp),
                                    loop_frequency(loop_frequency),
                                    vel_ph(vel_ph), 
                                    laser_sh(laser_sh), 
                                    odom_sh(odom_sh), 
                                    pid_ctr(pid_ctr),
                                    m_logger(logpath) 
{
}

VelController::~VelController()
{
}

//** Main algorithms **// 

bool VelController::omnidirectional_obstacle_check(const tf::Vector3& lv, 
                                                   const tf::Vector3& av,
                                                   const std::string& input_vel_frame_id,
                                                   const sensor_msgs::LaserScan& laser_msg,
                                                    geometry_msgs::TwistStamped& cmd_vel_msg)
{
    //! This transformation still needs work
    //! For now we trivially assume that input_vel_frame_id = working_frame_id
    if (!update_transform(this->working_frame_id, input_vel_frame_id)) {
        m_logger.logstr(std::string("omnidirectional_obstacle_check:") + 
                        std::string("Cannot get input_vel_frame -> working_frame transform.\n"));
        return false;
    }
    //!
    tf::Vector3 wf_lv = T(lv);
    tf::Vector3 wf_av = T(av); 
    // Now transform from working frame to output frame (i.e. "odom" for mavros)
    if (!update_transform(output_vel_frame_id, this->working_frame_id)) {
        m_logger.logstr(std::string("omnidirectional_obstacle_check:") + 
                        std::string("Cannot get working_frame_id -> output_vel_frame_id transform."));
        return false;
    }
    tf::Vector3 ovf_lv = T(wf_lv);
    tf::Vector3 ovf_av = T(wf_av);
    // Fill in angular velocity and headers of message regardless of obstacles 
    vector3TFToMsg(ovf_av, cmd_vel_msg.twist.angular);
    cmd_vel_msg.header.frame_id = output_vel_frame_id;
    // Identify requested direction of movement wrt working frame
    double dir_ang = atan2(wf_lv.getY(), wf_lv.getX());
    int min_index = 0, max_index = 0;
    ranges_indices(laser_msg, dir_ang - (osp.ang_range / 2.0), dir_ang + (osp.ang_range / 2.0),
                    min_index, max_index);
    //! Still weak iteration: see min_distance for more extensive checks
    for (int i = min_index; i < max_index; i++) {
        // If one single obstacle detected within threshhold distance, stop drone
        if (laser_msg.ranges[i] < osp.distance && is_within_range(laser_msg, i)) {
            // If obstacle too close, return false without filling linear component
            #ifdef OBSTLOG
            m_logger.logstr( std::string("omnidirectional_obstacle_check: ") + 
                             std::string("Obstacle detected within thresh_distance ") +
                    std::to_string(osp.distance) + " at laser scan angle "  +
                    std::to_string(i*laser_msg.angle_increment + laser_msg.angle_min) +
                    std::string(" and distance ") + std::to_string(laser_msg.ranges[i]) + 
                    std::string("\n") );
            #endif //OBSTLOG
            return false;
        }
    }
    // If no obstacle is detected, fill up also linear part of message
    vector3TFToMsg(ovf_lv, cmd_vel_msg.twist.linear);
    return true;
}

void VelController::follow_wall(const double dt, 
                            //! Drone translation after PID failure
                                const double side_vel,
                            //! Optional parameter to assess pid success/ failure
                                bool& pid_success)                  
{
    // Local variables used later
    geometry_msgs::TwistStamped cmd_vel_msg;
    tf::Vector3 obst_pos;
    std::string obst_frame_id;
    // Compute current minimum wall distance
    laser_sh->update_msg();
    const sensor_msgs::LaserScan laser_msg = laser_sh->currMsg();
    double curr_wall_distance = min_wall_distance(laser_msg, wsp);
    // If no wall detected on the right, flag PID failure & handle missed wall
    if (curr_wall_distance >= laser_msg.range_max) {
        m_logger.logstr("follow_wall: lost wall contact, rotating...\n");
        pid_success = false; // Flag PID failure
        cmd_vel_msg.twist.angular.z = angular_velocity;
        ros::Rate rate(loop_frequency);
    //! Assumes that obstacle is still within sensor range!
        while (ros::ok() && !get_closest_obstacle_position(obst_pos, obst_frame_id)) {
            vel_ph->publish(cmd_vel_msg);
            rate.sleep();
        }
        m_logger.logstr("follow_wall: got wall contact back, handling missed wall...\n");
        handle_missed_wall(obst_pos, obst_frame_id, side_vel);
        return;
    }
    // Compute resulting error
    double next_error = wsp.distance - curr_wall_distance;
    // Feed error in PID controller
    tf::Vector3 pid_av (0.0, 0.0, pid_ctr->step(next_error, dt));
    ros::Time startTime = ros::Time::now();
    ros::Duration loopDuration (dt); // dt seconds 
    ros::Rate rate(loop_frequency);  // set up frequency to control loop iteration
    // Let the drone move throughout the given time step
    while (ros::ok() && ((ros::Time::now() < startTime + loopDuration))) {
        // As soon as obstacle check fails, handle close wall and break loop 
        if (!omnidirectional_obstacle_check(linear_velocity, pid_av, working_frame_id, 
                                            laser_sh->currMsg(), cmd_vel_msg))
        {
            pid_success = false; // Flag PID failure
            m_logger.logstr("follow_wall: detected close obstacle, handling missed wall...\n");
            // Get the position of the closest obstacle
            tf::Vector3 obst_pos;
            std::string obst_frame_id;
            // We assume this function to return successfully, since an obstacle wa sjust detected
            get_closest_obstacle_position(obst_pos, obst_frame_id);
            // Handle the obstacle
            handle_missed_wall( obst_pos, obst_frame_id, side_vel);
            return; 
        }
        // Stamp & publish the resulting message
        cmd_vel_msg.header.stamp = ros::Time::now();
        vel_ph->publish(cmd_vel_msg);
        // Update laser scan at each iteration
        laser_sh->update_msg();
        rate.sleep(); 
    }
}

//** Overloaded copy **//
void VelController::follow_wall(const double dt, 
                                const double side_vel)
{
    bool pid_success = true;
    follow_wall(dt, side_vel, pid_success);
}

bool VelController::Bug2( const std::string& goal_frame_id,
                          const tf::Vector3& goal_point,     
                    //! Parameters for PID
                          const PIDparams& pid_params,
                    //! Parameters for follow_wall phase
                          const double dt, 
                          const double side_vel)
{
    // Retrieve starting position (in odom frame)
    tf::Vector3 start;
    update_position_vector(start);
    // Transform goal point to odometry frame
    if (!update_transform(odom_sh->currMsg().header.frame_id, goal_frame_id)) {
        m_logger.logstr("Cannot get goal_frame -> odometry transform.");
        return false;
    }
    tf::Vector3 goal = T(goal_point);
        m_logger.logstr( "goal_frame_id transformed to " + odom_sh->currMsg().header.frame_id + 
                        " ; goal point to ( "  + std::to_string(goal.getX()) + ", " 
                        +  std::to_string(goal.getY()) + ", " + std::to_string(goal.getZ()) + " )\n");   
    // Rotate toward goal point
    rotateYaw( atan2(goal.getY() - start.getY(), goal.getX() - start.getX()), 
                ang_tol, angular_velocity, loop_frequency);
    ros::Rate rate (loop_frequency);
    geometry_msgs::TwistStamped cmd_vel_msg;
    tf::Vector3 curr_pos;
    tf::Vector3 obst_pos;
    bool close_obstacle = false;
    // Move forwards until obstacle is encountered
    do {
        rate.sleep();
        update_position_vector(curr_pos);
        laser_sh->update_msg();
        // If no obstacle, move towards goal and continue loop
        if (omnidirectional_obstacle_check(linear_velocity, tf::Vector3(0.0, 0.0, 0.0), 
                    working_frame_id, laser_sh->currMsg(), cmd_vel_msg))
        {
            vel_ph->publish(cmd_vel_msg);
            continue;
        }
        // If we encountered obstacle on the way, circumnavigate it with tested PID parameters
        close_obstacle = true; // mark obstacle as close
        update_position_vector(curr_pos);
        memcpy(&obst_pos, &curr_pos, sizeof(tf::Vector3)); // Record starting position
        pid_ctr->reset( //! PID parameters
                        pid_params );   
        do {
            update_position_vector(curr_pos);
            follow_wall(dt, side_vel);
            // Check if we left the tolerance level from the line
            if (close_obstacle && !intersects_line(start, goal, curr_pos, point_tol)) {
                close_obstacle = false;
            }
            // Check if we intersect line after leaving obstacle,
            // and if we are closer to goal than before
        } while (ros::ok() && (close_obstacle || !intersects_line(start, goal, curr_pos, point_tol) ||
                                xy_distance(curr_pos, goal) > xy_distance(obst_pos, goal)));
        // After surpassing obstacle, rotate again towards goal
        rotateYaw( atan2(goal.getY() - curr_pos.getY(), goal.getX() - curr_pos.getX()), 
                    ang_tol, angular_velocity, loop_frequency);
        // Check if we reached goal (x,y) coordinates at the end of loop
    } while (ros::ok() && (absolute_value(curr_pos.getX() - goal_point.getX()) > point_tol.getX() ||
                           absolute_value(curr_pos.getY() - goal_point.getY()) > point_tol.getY()) );
    return true;
}

bool VelController::ZN_tuning_test( std::string  outfile,
                                    const double Kp_start,
                                    const double Kp_end,
                                    const double Kp_step,
                                    const double time_goal,
                                //! Parameters for follow_wall()
                                    const double dt, 
                                    const double side_vel)
{
    bool success = false; // whether at least one pid configuration passed the test 
    m_logger.logstr("Starting ZN_tuning_test\n" );
    // Initialize proportional PID parameter
    double Kp_curr = Kp_start;
    // Run tests for all desired Kp values
    while (ros::ok() && (Kp_curr < Kp_end)) {
        sleep(1); // Sleep between tuning tests to allow updating of subscribers
        m_logger.logstr("Starting ZN_tuning_test for Kp = " + std::to_string(Kp_curr) + "\n" );
        pid_ctr->reset( PIDparams(0.0, Kp_curr, 0.0, 0.0, -1));
        double startTime = ros::Time::now().toSec();
        while (ros::ok()) {
            double currTime = ros::Time::now().toSec();
            bool pid_success = true;
            follow_wall(dt, side_vel, pid_success);
            // Handle PID failure
            if (!pid_success) {
                m_logger.logstr("ZN_tuning_test failed for Kp = " + std::to_string(Kp_curr) +
                    + " after " + std::to_string( currTime - startTime) + " seconds.\n");
                break;
            }
            // Handle PID success
            if (currTime - startTime > time_goal) {
                m_logger.logstr("ZN_tuning_test succeeded for Kp = " + std::to_string(Kp_curr) + "\n" );
                success = true;
                break;
            }
        }
        // Log PID results to file
        pid_ctr->log2file(outfile);
        // Increment proportional term by desired step after each iteration
        Kp_curr += Kp_step;
    }
    m_logger.logstr("Ending ZN_tuning_test" );
    return success;
}

bool VelController::get_closest_obstacle_position(tf::Vector3& obst_pos,
                                                  std::string& obst_frame_id)
{
    // Perform a full scan and determine the angle at which the closest obstacle is located
    laser_sh->update_msg();
    const sensor_msgs::LaserScan laser_msg = laser_sh->currMsg();
    double min_dist_ang = 0.0;
    double min_dist     = min_distance(laser_msg, 
            laser_msg.angle_min, laser_msg.angle_max, min_dist_ang);
    if (min_dist_ang == 0.0) {
        m_logger.logstr("get_closest_obstacle_position: failed to detect an obstacle.\n" );
        return false;
    }
    // Get the resulting point in the drone's working frame
    if (!update_transform(working_frame_id, input_laser_frame_id)) {
        m_logger.logstr(std::string("get_closest_obstacle_position: ") +
                        std::string("Cannot get input_laser_frame_id -> working_frame_id transform."));
        return false;
    }
    // IMPORTANT: Z-position ignored since laser sensor is only a 2-D lidar!
    tf::Vector3 obst_pos_wf = T(tf::Vector3( min_dist * cos(min_dist_ang),
                                             min_dist * sin(min_dist_ang),
                                             0.0));
    // Update reference parameters accordingly
    obst_pos      = obst_pos_wf;
    obst_frame_id = working_frame_id;
    m_logger.logstr(std::string("get_closest_obstacle_position: closest obtstacle detected at ") +
                    std::string("(") + std::to_string(obst_pos_wf.getX()) + std::string(", ") +
                                       std::to_string(obst_pos_wf.getY()) + std::string(", ") +
                                       std::to_string(obst_pos_wf.getZ()) + std::string(") ") + 
                    std::string("in ") + working_frame_id + std::string("\n") );
    return true;
}

bool VelController::handle_missed_wall( const tf::Vector3& obst_pos,
                                        const std::string& obst_frame_id,
                                    //! Drone translation after PID failure
                                        const double side_vel)
{
    stop(); // immediately
    // Get obstacle position in odom frame (to later compare with odometry msg)
    if (!update_transform(odom_sh->currMsg().header.frame_id, obst_frame_id)) {
        m_logger.logstr("handle_missed_wall: Cannot get obst_frame_id -> odom transform.");
        return false;
    }
    tf::Vector3 obst_pos_odom = T(obst_pos);
    m_logger.logstr("handle_missed_wall: Moving back to goal distance.\n" );
    // Use the assumption that the wall is on our right
    odom_sh->update_msg();
    const nav_msgs::Odometry odom_msg = odom_sh->currMsg();
    rotateYaw( atan2(obst_pos_odom.getY() - odom_msg.pose.pose.position.y,
                     obst_pos_odom.getX() - odom_msg.pose.pose.position.x) + M_PI/2,
                ang_tol, angular_velocity, loop_frequency);
    // Record starting wall distance
    laser_sh->update_msg();
    double start_wd = min_wall_distance(laser_sh->currMsg(), wsp);
    double curr_wd  = start_wd;
    // Identify necessary direction of motion to reach goal distance from wall
    double y_vel   = (wsp.distance > start_wd) ? side_vel : -side_vel;
    geometry_msgs::TwistStamped cmd_vel_msg; // prepare velocity message
    ros::Rate rate (loop_frequency);
    while (ros::ok() && (absolute_value(curr_wd - wsp.distance) // compare with xy-magnitude
                > std::sqrt( pow(point_tol.getX(), 2) + pow(point_tol.getY(), 2)))) {
        rate.sleep();
        // Even if we assume no obstacle, this function is still necessary
        // to translate commands to appropriate frame
        laser_sh->update_msg();
        const sensor_msgs::LaserScan laser_msg = laser_sh->currMsg();
        omnidirectional_obstacle_check( tf::Vector3(0.0, y_vel, 0.0),
                                        tf::Vector3(0.0, 0.0,   0.0),
                                        working_frame_id,
                                        laser_msg,
                                        cmd_vel_msg);
        vel_ph->publish(cmd_vel_msg);
        // Update current wall distance
        curr_wd = min_wall_distance(laser_msg, wsp);
    }
    stop();
    m_logger.logstr("handle_missed_wall: Moved back to goal distance.\n" );
    return true;
}

void VelController::reset_PID( const PIDparams nparams) 
{
    // Reset PID with desired values
    pid_ctr->reset( nparams);
}

//** Private helper functions **//

inline void VelController::ranges_indices(const sensor_msgs::LaserScan& laser_msg,
                                          const double min_scan_angle,
                                          const double max_scan_angle,
                                              int& min_index,
                                              int& max_index)
{
    min_index = std::max((int)floor((min_scan_angle - laser_msg.angle_min) / laser_msg.angle_increment) , 0);
    max_index = std::min((int)ceil((max_scan_angle - laser_msg.angle_min) / laser_msg.angle_increment), 
        (int)((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment));
}

inline void VelController::bring2range( const double angle_min,
                                        const double angle_max,
                                        double& angle)
{
    while (angle > angle_max || angle < angle_min) {
        angle += (2*M_PI) * ((angle < angle_min) - (angle > angle_max));
    }
}       

bool VelController::is_within_range(const sensor_msgs::LaserScan& laser_msg, const int i)
{
    return (laser_msg.ranges[i] > laser_msg.range_min &&
            laser_msg.ranges[i] < laser_msg.range_max);
}

double VelController::min_distance(const sensor_msgs::LaserScan& laser_msg,
                                   double min_scan_angle, 
                                   double max_scan_angle,
                                //! Optional parameter keeping angle of minimum distance
                                   double& min_dist_ang)
{
    //! This function is quite involved, so follow steps carefully
    /* 1) Bring the scan angles within the desired range for the sensor */ 
    bring2range(laser_msg.angle_min, laser_msg.angle_max, min_scan_angle);
    bring2range(laser_msg.angle_min, laser_msg.angle_max, max_scan_angle);
    /* 2) Retrieve the corresponding min & man indexes in ranges[] */ 
    int min_index, max_index;
    ranges_indices(laser_msg, min_scan_angle, max_scan_angle, min_index, max_index);
    /* 3) Because at this point we have no guarantee that min_index < max_index
    *     due to the discontinuity in the angle domain, we have to perform two iterations,
    *     which will be reduntant whenever min_index < max_index  */
    int num_readings = ceil((double)(laser_msg.angle_max - laser_msg.angle_min) / 
                            (double)laser_msg.angle_increment);
    int start_index  = (min_index < max_index) ? min_index : 0;
    int end_index    = (min_index < max_index) ? max_index : num_readings;
    double min_distance = laser_msg.range_max; // Initialize minimum distance at highest value
    // Now iterate through full array
    for (int i = 0; i < num_readings; i++) {
        if (((i >= start_index && i < max_index) ||
             (i >= min_index   && i < end_index))  // inclusion conditions
            && laser_msg.ranges[i] < min_distance  // new minimum condition
            && is_within_range(laser_msg, i))      // reading validity condition
        {
            min_distance = laser_msg.ranges[i]; // Set minimum distance & corresponding angle
            min_dist_ang = laser_msg.angle_min + ((double) i) * laser_msg.angle_increment;
        }
    }
    return min_distance;
}

//** Overloaded copy **//
double VelController::min_distance(const sensor_msgs::LaserScan& laser_msg,
                                   double min_scan_angle, 
                                   double max_scan_angle)
{
    double min_dist_ang = 0.0;
    return min_distance(laser_msg, min_scan_angle, max_scan_angle, min_dist_ang);
}

double VelController::min_wall_distance( const sensor_msgs::LaserScan& laser_msg,
                                         const ScanParameters& wsp)
{
    // Compute the desired scanning boundary angles
    const double min_scan_angle = wsp.ang_midpoint - (wsp.ang_range / 2.0);
    const double max_scan_angle = wsp.ang_midpoint + (wsp.ang_range / 2.0);
    // Compute current minimum wall distance
    return min_distance(laser_msg, min_scan_angle, max_scan_angle);
}

bool VelController::update_transform(const std::string& target_frame, 
                                     const std::string& source_frame)
{
    try {
        listener.lookupTransform(target_frame, source_frame, ros::Time(0), this->T);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }
    return true;
}

void VelController::getRPY(const nav_msgs::Odometry& odom_msg, 
                            double& roll,
                            double& pitch,
                            double& yaw)
{
    tf::Quaternion quat;
    quaternionMsgToTF(odom_msg.pose.pose.orientation, quat);
    tf::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);
}

double VelController::absolute_value(double x)
{
    return ((x >= 0) ? x : -x);
}

bool VelController::is_close( const tf::Vector3& v1,
                              const tf::Vector3& v2,
                              const tf::Vector3& tol)
{
    return ( absolute_value(v1.getX() - v2.getX()) < tol.getX()
          && absolute_value(v1.getY() - v2.getY()) < tol.getY()
          && absolute_value(v1.getZ() - v2.getZ()) < tol.getZ());
}

void VelController::stop()
{
    geometry_msgs::TwistStamped cmd_vel_msg;
    vel_ph->publish(cmd_vel_msg);
    m_logger.logstr( "Stopped drone\n");
}

void VelController::rotateYaw( const double input_yaw, 
                               const double tol, 
                               const double ang_vel,  
                               int frequency)
{
    m_logger.logstr( "rotateYaw: input yaw: " + std::to_string(input_yaw) + "\n");
    // Bring input_yaw within desired angular range for the sensor
    laser_sh->update_msg();
    const sensor_msgs::LaserScan laser_msg = laser_sh->currMsg();
    double output_yaw = input_yaw;
    bring2range(laser_msg.angle_min, laser_msg.angle_max, output_yaw);
    // Prepare velocity message
    geometry_msgs::TwistStamped cmd_vel_msg;
    cmd_vel_msg.twist.angular.z = ang_vel; // set z-component of angular velocity only
    double roll, pitch, yaw;
    ros::Rate rate (frequency);
    do {
        odom_sh->update_msg(); // update yaw measurement
        getRPY( odom_sh->currMsg(), roll, pitch, yaw );
        vel_ph->publish(cmd_vel_msg); // move by given angular velocity
        rate.sleep();
        #ifdef ROTLOG
        m_logger.logstr( "rotateYaw: current yaw: " + std::to_string(yaw) + "\n");
        #endif // ROTLOG

    } while (ros::ok() && absolute_value(output_yaw - yaw) > tol);
    stop();
    m_logger.logstr( "rotateYaw: output yaw: " + std::to_string(yaw) + "\n");
}

bool VelController::intersects_line( const tf::Vector3& start,
                                     const tf::Vector3& goal,
                                     const tf::Vector3& curr_pos,
                                     const tf::Vector3& tol)
{
    // Determine gradient and y-intersect of the line
    double m = (goal.getY() - start.getY()) / (goal.getX() - start.getX());
    double c = start.getY() - m * start.getX();
    // Verify if sufficiently close in y-direction
    return absolute_value((curr_pos.getY() - (m * curr_pos.getX() + c))) < tol.getY();
}

void VelController::update_position_vector( tf::Vector3& v) 
{
    odom_sh->update_msg();
    pointMsgToTF(odom_sh->currMsg().pose.pose.position, v);
}

double VelController::xy_distance( tf::Vector3& v1, tf::Vector3& v2)
{
    return std::sqrt( pow(v2.getX() - v1.getX(), 2) +
                      pow(v2.getY() - v1.getY(), 2));
}

