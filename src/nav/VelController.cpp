
#include "VelController.h"

//** Contructor & destructor **//
VelController::VelController(PubHandler <geometry_msgs::TwistStamped> * const vel_ph,
                            SubHandler <sensor_msgs::LaserScan>      * const laser_sh,
                            SubHandler <nav_msgs::Odometry>          * const odom_sh,
                            PID* pid_ctr,
                            //! Navigation parameters
                            const double         linear_speed,
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
                                    linear_speed(linear_speed),
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
                                                    geometry_msgs::TwistStamped& cmd_vel_msg,
                                                    bool unchecked)
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
        if (laser_msg.ranges[i] < osp.distance && is_within_range(laser_msg, i)
            // If requested with the unchecked parameter, each linear velocity command goes through
            && !unchecked) {
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

void VelController::handle_lost_wall_contact( tf::Vector3& obst_pos,
                                              std::string& obst_frame_id)
//! This fucntion needs to be made way more robust and reliable
{
    m_logger.logstr("handle_lost_wall_contact: rotating...\n");
    geometry_msgs::TwistStamped cmd_vel_msg;
    cmd_vel_msg.twist.angular.z = angular_velocity;
    ros::Rate rate(loop_frequency);
//! Assumes that obstacle is still within sensor range!
    /** IMPORTANT: NOT a robust approach, since laser may detect something (error) too soon! 
     * Hence, repeating the query twice for redundancy */
    do {
        vel_ph->publish(cmd_vel_msg);
        rate.sleep();
    } while (ros::ok() && !get_closest_obstacle_position(obst_pos, obst_frame_id));
    // Stop the drone for a few milliseconds and call the function again redundantly
    stop();
    get_closest_obstacle_position(obst_pos, obst_frame_id, true, 2);
    m_logger.logstr("handle_lost_wall_contact: got wall contact back...\n");
}

bool VelController::follow_wall(const double dt, 
                                const double side_vel,
                                const double orbit_angle,
                                const double climb_angle,
                                bool& wall_contact,
                                bool& close_obstacle)                  
{
    // Local variables
    geometry_msgs::TwistStamped cmd_vel_msg;
    sensor_msgs::LaserScan      laser_msg;
    nav_msgs::Odometry          odom_msg;
    tf::Vector3 obst_pos;
    std::string obst_frame_id;
    tf::Vector3 pid_lv;
    tf::Vector3 pid_lv_ovf;
    tf::Vector3 pid_av;
    // Check if we still have wall contact
    laser_sh->update_msg();
    laser_msg = laser_sh->currMsg();
    double curr_wall_distance = min_wall_distance(laser_msg, wsp);
    if (curr_wall_distance >= laser_msg.range_max) {
        m_logger.logstr("follow_wall: lost wall contact\n");
        wall_contact = false; // Flag lost wall contact
        return false;
    }
    // Attempt to retrieve the tangential angle in the odometry reference frame (NO redundancy here)
    get_closest_obstacle_position(obst_pos, obst_frame_id);
    double tang_ang = get_wall_tangential_angle(obst_pos, obst_frame_id);
    if (tang_ang == -1.0) {
        m_logger.logstr("follow_wall: Error detecting tangential angle\n");
        return false;
    }
    // Compute resulting error, and resulting PID output
    // The PID output is interpreted as the deviation from the tangential angle
    double next_error = wsp.distance - curr_wall_distance;
    double vel_ang    = tang_ang + pid_ctr->step(next_error, dt); // after an increment angle is not guaranteed to be in range!
    bring2range(laser_msg.angle_min, laser_msg.angle_max, vel_ang); 
    // Compute the resulting velocity vector in the global fixed frame
    // This vector will remain constant throughout the entire PID iteration
    double v_xy = linear_speed * cos(climb_angle);  // magnitude of velocity's xy-plane projection
    pid_lv_ovf.setZ( linear_speed * sin(climb_angle)); // vertical velocity component
    pid_lv_ovf.setX( v_xy * cos(vel_ang));
    pid_lv_ovf.setY( v_xy * sin(vel_ang));
    // Get the corresponding transform in the base_link frame
    // This will be modified through the iteration to adjust the drone's yaw value
    if (!update_transform(working_frame_id, output_vel_frame_id)) {
        m_logger.logstr(std::string("follow_wall: ") +
                        std::string("Cannot get output_vel_frame_id -> working_frame_id transform."));
        return false;
    }
    pid_lv = T(pid_lv_ovf);
    // Determine the orbital angle discrepancy
    odom_sh->update_msg();
    odom_msg = odom_sh->currMsg();
    double roll, pitch, yaw;
    getRPY(odom_msg, roll, pitch, yaw);
    // Handle the discontinuity between yaw and vel_ang
    double curr_orbit_ang = yaw - vel_ang;
    curr_orbit_ang += 2*M_PI * ((curr_orbit_ang < -M_PI) - (curr_orbit_ang > M_PI));
    const double ang_discr = orbit_angle - curr_orbit_ang; // discrepancy wrt the desired orbiting angle
    // Log current orbiting angle and discrepancy to assess precision
    m_logger.logstr(std::string("follow_wall: curr_orbit_ang = ") + std::to_string(curr_orbit_ang) + std::string("\n"));
    m_logger.logstr(std::string("follow_wall: ang_discr = ")      + std::to_string(ang_discr)      + std::string("\n"));
    // Compute and set a reasonable angular velocity to adjust for this angular discrepancy
        //! Could also use a second PID controller here
    double ang_vel = (ang_discr / dt); // notice that this is a simple P controller,
    pid_av.setZ(ang_vel);   // aiming to bring the orbiting angle to the desired value by the end of this PID-iteration
    // Compute the phase shift for the SHM oscillations on the xy-plane, using starting base_link velocity
    // The - accounts for the y-axis of base_link going to the "left" wrt the corresponding x-axis
    const double b = atan2(-pid_lv.getY(), pid_lv.getX()) / ang_vel;
    // Set up loop iteration
    double startTime = ros::Time::now().toSec();
    double t = 0.0; // variable measuring iteration time
    ros::Rate rate(loop_frequency);  // set up frequency to control loop iteration
    // Let the drone move throughout the given time step
    while (ros::ok() && (t < dt)) {
        t = ros::Time::now().toSec() - startTime; // update iteration time
        laser_sh->update_msg(); // retrieve most recent laser message
        laser_msg = laser_sh->currMsg();
        // Modify the working_frame velocity vector xy components to adjust the orbiting angle
        // whilst keeping the overall fixed frame velocity vector constant
        // This is done by using SHM equations as seen in rotate_in_line()
        pid_lv.setX( v_xy *  cos(ang_vel * (t + b))); // the - accounts for the y-axis of base_link
        pid_lv.setY( v_xy * -sin(ang_vel * (t + b))); // going "left" wrt the x-axis of base_link
        // As soon as obstacle check fails, handle close wall and break loop 
        if (!omnidirectional_obstacle_check(pid_lv, pid_av, working_frame_id, 
            /** NOTE: disabling obstacle detection for now (gives too many false obstacles) **/
                                            laser_msg, cmd_vel_msg, true)) 
        {
            m_logger.logstr("follow_wall: detected close obstacle\n");
            close_obstacle = true; // flag failure due to a close obstacle
            return false;             
        }
        // Stamp & publish the resulting message
        cmd_vel_msg.header.stamp = ros::Time::now();
        vel_ph->publish(cmd_vel_msg);
        rate.sleep(); 
    }
    return true;
}

//** Overloaded copy **//
bool VelController::follow_wall(const double dt, 
                                const double side_vel,
                                const double orbit_angle,
                                const double climb_angle)                 
{
    bool wall_contact   = true;
    bool close_obstacle = false;
    return follow_wall(dt, side_vel, orbit_angle, climb_angle, 
                        wall_contact, close_obstacle);
}

double VelController::get_wall_tangential_angle(const tf::Vector3& obst_pos,
                                                const std::string& obst_frame_id)
{
    // First ensure that we are working in the odometry frame
    odom_sh->update_msg();
    nav_msgs::Odometry odom_msg = odom_sh->currMsg();
    if (!update_transform(odom_msg.header.frame_id, obst_frame_id)) {
        m_logger.logstr("get_wall_tangential_angle: Cannot get obst_frame_id -> odom transform.");
        return -1.0;
    }
    tf::Vector3 obst_pos_odom = T(obst_pos);
    //! Use the assumption that the wall is on our right
    double tang_ang = atan2(obst_pos_odom.getY() - odom_msg.pose.pose.position.y,
                            obst_pos_odom.getX() - odom_msg.pose.pose.position.x) + M_PI/2;
    // Ensure the angle is still within sensor range after increment
    bring2range(laser_sh->currMsg().angle_min, laser_sh->currMsg().angle_max, tang_ang); 
    return tang_ang;
}

bool VelController::Bug2( const std::string& goal_frame_id,
                          const tf::Vector3& goal_point,     
                    //! Parameters for PID
                          const PIDparams& pid_params,
                    //! Parameters for follow_wall phase
                          const double dt, 
                          const double side_vel,
                          const double orbit_angle,
                          const double climb_angle)
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
        if (omnidirectional_obstacle_check(tf::Vector3(linear_speed, 0.0, 0.0), 
                                           tf::Vector3(0.0, 0.0, 0.0), 
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
            follow_wall(dt, side_vel, orbit_angle, climb_angle);
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
                                    const double side_vel,
                                    const double orbit_angle,
                                    const double climb_angle)
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
            // Flag variables for error handling (reset on each loop iteration)
            bool wall_contact   = true;
            bool close_obstacle = false;
            follow_wall(dt, side_vel, orbit_angle, climb_angle, wall_contact, close_obstacle);
            // Determine whether pid was successful based on this output 
            bool pid_success = wall_contact && !close_obstacle;
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

void VelController::rotate_in_line( const tf::Vector3& linear_vel_input,
                                    const std::string& input_vel_frame_id,
                                    const double ang_vel,
                                    const double exec_time)
{
    // Transform the input velocity vector to the working frame to account for 
    // the current yaw value of the drone
    if (!update_transform(working_frame_id, input_vel_frame_id)) {
        m_logger.logstr(std::string("rotate_in_line: ") +
                        std::string("Cannot get input_vel_frame_id -> working_frame_id transform."));
        return;
    }
    tf::Vector3 linear_vel = T(linear_vel_input); // velocity magnitudes are preserved
    // Compute the phase shift for the SHM oscillations on the xy-plane
    // The - accounts for the y-axis of base_link going to the "left" wrt the corresponding x-axis
    const double b = atan2(-linear_vel.getY(), linear_vel.getX()) / ang_vel;
    // Compute the 2-D magnitude of the velocity
    const double v = xy_distance( linear_vel, tf::Vector3(0.0, 0.0, 0.0));
    // Prepare for loop iteration
    geometry_msgs::TwistStamped cmd_vel_msg;
    sensor_msgs::LaserScan laser_msg;
    double startTime = ros::Time::now().toSec();
    double t = 0.0; // variable measuring time elapsed
    ros::Rate rate (loop_frequency);
    while (ros::ok() && (t < exec_time)) {
        laser_sh->update_msg(); 
        laser_msg = laser_sh->currMsg(); // retrieve most recent laser reading
        t  = ros::Time::now().toSec() - startTime; // update iteration time
        // Prepare twist message, using parameters derived from
        // map->base_link transform and SHM equations
        omnidirectional_obstacle_check(
            tf::Vector3(v *  cos(ang_vel * (t + b)), // the - accounts for the y-axis of base_link
                        v * -sin(ang_vel * (t + b)), // going "left" wrt the x-axis of base_link
                        linear_vel.getZ()), // keep the z-component of velocity unchanged                        
            tf::Vector3(0.0, 0.0, ang_vel), // we only work on 1 d.o.f. for angular velocity
            working_frame_id, // the function will handle the transformation to the odom frame
            laser_msg,
            cmd_vel_msg);
        vel_ph->publish(cmd_vel_msg);
        rate.sleep();
    }
}

bool VelController::get_closest_obstacle_position(tf::Vector3& obst_pos,
                                                  std::string& obst_frame_id,
                                                  bool redundant,
                                                  int  num_calls,
                                                  int  sleep_ms)
{
    // Execute redundant calls
    //! Not an appropriate method to handle laser readings noise!!
    if (redundant) {
        int succ_calls = 0;
        while (succ_calls < num_calls) {
            usleep(sleep_ms); // to update laser reading and transform
            succ_calls += get_closest_obstacle_position(obst_pos, obst_frame_id);
        }
    }
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
                                        const double side_vel,
                                        const double orbit_angle,
                                        const bool final_rot_adjustment)
{
    stop(); // immediately
    double tang_ang = get_wall_tangential_angle(obst_pos, obst_frame_id);
    if (tang_ang == -1.0) {
        m_logger.logstr("handle_missed_wall: Error retrieving tangential angle to wall.\n" );
        return false;
    }
    // Rotate back to the desired orbit_angle
    rotateYaw( (tang_ang + orbit_angle), ang_tol, angular_velocity, loop_frequency);
    // Get the required direction directly in the odometry frame
    tf::Vector3 ovf_lv (side_vel * cos(tang_ang + M_PI/2.0),
                        side_vel * sin(tang_ang + M_PI/2.0),
                        0.0);
    m_logger.logstr( std::string("handle_missed_wall: Moving (") +
                     std::to_string(ovf_lv.getX()) + std::string(", ") +
                     std::to_string(ovf_lv.getY()) + std::string(", ") +
                     std::to_string(ovf_lv.getZ()) + std::string(") ") +
                     std::string("with respect to output_velocity_frame.\n"));
    // Record starting wall distance
    laser_sh->update_msg();
    double start_wd = min_wall_distance(laser_sh->currMsg(), wsp);
    double curr_wd  = start_wd;
    double dir      = (wsp.distance > start_wd) ? 1.0 : -1.0;
    geometry_msgs::TwistStamped cmd_vel_msg;
    ros::Rate rate (loop_frequency);
    m_logger.logstr(std::string("handle_missed_wall: Moving back to goal distance.") +
                    std::string("with dir = ") + std::to_string(dir) + std::string("\n"));
    while (ros::ok() && (absolute_value(curr_wd - wsp.distance) 
            // compare current distance with xy-magnitude of tolerance vector
            > xy_distance(point_tol, tf::Vector3(0.0, 0.0, 0.0)))) {
        rate.sleep();
        laser_sh->update_msg();
        const sensor_msgs::LaserScan laser_msg = laser_sh->currMsg();
        // Move radially away from the obstacle, using computed velocity
        omnidirectional_obstacle_check( tf::Vector3(dir * ovf_lv.getX(), 
                                                    dir * ovf_lv.getY(),
                                                    dir * ovf_lv.getZ()),
                                        tf::Vector3(0.0, 0.0, 0.0),
                                        output_vel_frame_id,
                                        laser_msg,
                                        cmd_vel_msg);
        vel_ph->publish(cmd_vel_msg);
        // Update current wall distance
        curr_wd = min_wall_distance(laser_msg, wsp);
    }
    stop();
    m_logger.logstr("handle_missed_wall: Moved back to goal distance.\n" );
    // For further precision, we can update again the pose of the drone
    if (final_rot_adjustment) 
    {
        // Try to update the obstacle position
        tf::Vector3 new_obst_pos;
        std::string new_obst_frame_id;
        if (!get_closest_obstacle_position(new_obst_pos, new_obst_frame_id, true, 2)) {
            m_logger.logstr(std::string("handle_missed_wall:)") +
                            std::string("Error updating closest obstacle position. Using starting value.\n" ));
            new_obst_pos      = obst_pos;
            new_obst_frame_id = obst_frame_id;
        }
        // Rotate back to the desired orbiting angle
        rotateYaw( (get_wall_tangential_angle(new_obst_pos, new_obst_frame_id) + orbit_angle),
                    ang_tol, angular_velocity, loop_frequency);
        m_logger.logstr("handle_missed_wall: Rotated back to orbiting_angle.\n" );
    }
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

double VelController::xy_distance( const tf::Vector3& v1, 
                                   const tf::Vector3& v2)
{
    return std::sqrt( pow(v2.getX() - v1.getX(), 2) +
                      pow(v2.getY() - v1.getY(), 2));
}

double VelController::xyz_distance( const tf::Vector3& v1, 
                                    const tf::Vector3& v2)
{
    return std::sqrt( pow(v2.getX() - v1.getX(), 2) +
                      pow(v2.getY() - v1.getY(), 2) +
                      pow(v2.getZ() - v1.getZ(), 2));
}

