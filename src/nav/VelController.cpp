
#include "VelController.h"

VelController::VelController(PubHandler <geometry_msgs::TwistStamped> * const vel_ph,
                            SubHandler <sensor_msgs::LaserScan>       * const laser_sh,
                            SubHandler <nav_msgs::Odometry>           * const odom_sh,
                            SubHandler <sensor_msgs::Range>           * const range_sh,
                            PID * obst_pid,
                            PID * alt_pid,
                            const double         linear_speed,
                            const tf::Vector3    point_tol,
                            const double         angular_velocity,
                            const double         ang_tol,
                            const ScanParameters osp,
                            const ScanParameters wsp,
                            const int            loop_frequency,
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
                                    range_sh(range_sh),
                                    obst_pid(obst_pid),
                                    alt_pid(alt_pid),
                                    m_logger(logpath) 
{
}

VelController::~VelController()
{
}

bool VelController::omnidirectional_obstacle_check(const tf::Vector3& lv, 
                                                   const tf::Vector3& av,
                                                   const std::string& input_vel_frame_id,
                                                   const sensor_msgs::LaserScan& laser_msg,
                                                   geometry_msgs::TwistStamped& cmd_vel_msg,
                                                   bool unchecked)
{
    // The transformation to the working frame is necessary to compare the requested
    // direction with the laser scan message 
    if (!update_transform(this->working_frame_id, input_vel_frame_id)) {
        m_logger.logstr(std::string("omnidirectional_obstacle_check:") + 
                        std::string("Cannot get input_vel_frame -> working_frame transform.\n"));
        return false;
    }
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
    // Compute the minimum distance in the given angular range
    double min_scan_angle = dir_ang - (osp.ang_range / 2.0);
    double max_scan_angle = dir_ang + (osp.ang_range / 2.0);
    double min_dist = min_distance(laser_msg, min_scan_angle, max_scan_angle);
    // If requested with the unchecked parameter, each linear velocity command goes through
    if ((min_dist < osp.distance) && !unchecked) {
        #ifdef OBSTLOG
        m_logger.logstr(std::string("omnidirectional_obstacle_check: ") + 
                        std::string("Obstacle detected within thresh_distance ") +
                        std::to_string(osp.distance) + std::string("at distance ") +
                        std::to_string(min_dist)     + std::string("\n"));
        #endif //OBSTLOG
        return false;
    }
    // If no obstacle is detected, fill up also linear part of message
    vector3TFToMsg(ovf_lv, cmd_vel_msg.twist.linear);
    return true;
}

bool VelController::handle_lost_wall_contact( tf::Vector3& obst_pos,
                                              std::string& obst_frame_id,
                                              double dur)
{
    bool got_wall_contact = false;
    geometry_msgs::TwistStamped cmd_vel_msg;
    cmd_vel_msg.twist.angular.z = angular_velocity;
    double startTime = ros::Time::now().toSec();
    ros::Rate rate(loop_frequency);
    m_logger.logstr("handle_lost_wall_contact: rotating...\n");
    /** NOTE: Assumes that obstacle is still within sensor range! */ 
    while (ros::ok() && !get_closest_obstacle_position(obst_pos, obst_frame_id)) {
        // Check if time expired
        if (ros::Time::now().toSec() - startTime > dur) {
            m_logger.logstr(std::string("handle_lost_wall_contact: " +
                            std::string("failed to get wall contact within time duration.\n")));
            goto END;
        }
        vel_ph->publish(cmd_vel_msg);
        rate.sleep();
    }
    got_wall_contact = true;
    m_logger.logstr("handle_lost_wall_contact: got wall contact back...\n");
    END:
    stop();
    return got_wall_contact;
}

bool VelController::follow_wall(const double dt, 
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
    tf::Vector3 pid_av;
    // Check if we still have wall contact
    laser_sh->update_msg();
    laser_msg = laser_sh->currMsg();
    double curr_wall_distance = min_wall_distance(laser_msg, wsp);
    if (curr_wall_distance == laser_msg.range_max) {
        m_logger.logstr("follow_wall: lost wall contact\n");
        wall_contact = false; // Flag lost wall contact
        return false;
    }
    // Attempt to retrieve the tangential angle in the odometry reference frame
    double tang_ang;
    if (!get_closest_obstacle_position(obst_pos, obst_frame_id) ||
        (tang_ang = get_wall_tangential_angle(obst_pos, obst_frame_id)) == -1.0)
    {
        m_logger.logstr("follow_wall: Error detecting tangential angle\n");
        return false;
    }
    // Compute resulting error, and resulting PID output
    // The PID output is interpreted as the deviation from the tangential angle
    const double next_error = wsp.distance - curr_wall_distance;
    double vel_ang    = tang_ang + obst_pid->step(next_error, dt);
    // Each time you increment an angle, ensure it is within range:
    bring2range(laser_msg.angle_min, laser_msg.angle_max, vel_ang); 
    // Compute the resulting velocity vector in the global fixed frame
    // This vector will remain constant throughout the entire PID iteration
    const double v_xy = linear_speed * cos(climb_angle);  // magnitude of velocity's xy-plane projection
    const tf::Vector3 pid_lv_ovf (v_xy * cos(vel_ang),
                                  v_xy * sin(vel_ang),
                                  linear_speed * sin(climb_angle));
    // Get the corresponding transform in the base_link frame
    // This will be modified through the iteration to adjust the drone's yaw value
    if (!update_transform(working_frame_id, output_vel_frame_id)) {
        m_logger.logstr(std::string("follow_wall: ") +
                        std::string("Cannot get output_vel_frame_id -> working_frame_id transform."));
        return false;
    }
    pid_lv = T(pid_lv_ovf);
    // Determine the current orbiting angle
    odom_sh->update_msg();
    odom_msg = odom_sh->currMsg();
    double roll, pitch, yaw;
    getRPY(odom_msg, roll, pitch, yaw);
    double curr_orbit_ang = (yaw - vel_ang);
    // Handle the potential discontinuity between yaw and vel_ang
    curr_orbit_ang += (2*M_PI) * ((curr_orbit_ang < -M_PI) - (curr_orbit_ang > M_PI));
    const double ang_discr = orbit_angle - curr_orbit_ang; // discrepancy wrt the desired orbiting angle
    // Log precision
    m_logger.logstr(std::string("follow_wall: curr_orbit_ang = ") + std::to_string(curr_orbit_ang) + std::string("\n"));
    m_logger.logstr(std::string("follow_wall: ang_discr = ")      + std::to_string(ang_discr)      + std::string("\n"));
    // Compute and set a reasonable angular velocity to adjust for this angular discrepancy
    /** NOTE:  Could also use a full PID controller here for better precision */
    double ang_vel = (ang_discr / dt); // notice that this is a simple P controller,
    pid_av.setZ(ang_vel);   // aiming to bring the orbiting angle to the desired value by the end of this PID iteration
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
        if (!omnidirectional_obstacle_check(pid_lv, pid_av, working_frame_id, laser_msg, cmd_vel_msg)) 
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
                                const double orbit_angle,
                                const double climb_angle)                 
{
    bool wall_contact   = true;
    bool close_obstacle = false;
    return follow_wall(dt, orbit_angle, climb_angle, 
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
    /** NOTE: This is limited to the assumption of the wall being on our right! */
    double tang_ang = atan2(obst_pos_odom.getY() - odom_msg.pose.pose.position.y,
                            obst_pos_odom.getX() - odom_msg.pose.pose.position.x) + M_PI/2;
    // Ensure the angle is still within sensor range after incrementing
    bring2range(laser_sh->currMsg().angle_min, laser_sh->currMsg().angle_max, tang_ang); 
    return tang_ang;
}

void VelController::bug3_obstacle_handler(  const tf::Vector3& start_odom,
                                            const tf::Vector3& goal_odom, 
                                            const double       min_agl,
                                            const double       max_agl,
                                            double (*climb_ang_policy) (double, double, double),    
                                            const PIDparams& obst_pid_params,
                                            const double dt, 
                                            const double side_vel,
                                            const double orbit_angle)
{
    tf::Vector3 obst_pos;
    std::string obst_frame_id;
    sensor_msgs::Range range_msg; // keeping track of drone AGL at a given moment
    sensor_msgs::LaserScan laser_msg;
    double climb_angle = 0.0;    // climbing angle of the drone during obstacle circumvention
    tf::Vector3 curr_pos;       // current odometry position
    update_position_vector(curr_pos);
    const tf::Vector3 obst_start_pos (curr_pos); // position at which obstacle handling started 
    m_logger.logstr(std::string("bug3_obstacle_handler: Starting obstacle handling procedure at (") +
                    std::to_string(obst_start_pos.getX()) + std::string(", ") + 
                    std::to_string(obst_start_pos.getY()) + std::string(", ") +
                    std::to_string(obst_start_pos.getZ()) + std::string(")\n"));
    // Reset the obstacle pid parameters to the requested values
    obst_pid->reset(obst_pid_params);
    // Since it's difficult and unlikely to intersect a 3-D line, we apply the line-intersection condition as seen
    // for Bug2, projecting the line between start_odom and goal_odom on the xy-plane
    while (ros::ok() && !( intersects_line(start_odom, goal_odom, curr_pos, point_tol) &&
                           xy_distance(curr_pos, goal_odom) < xy_distance(obst_start_pos, goal_odom))) {
        range_sh->update_msg(); // use drone's altitude to determine climb angle
        range_msg = range_sh->currMsg();
        if (!is_within_range(range_msg)) {
            m_logger.logstr(std::string("bug3_obstacle_handler: Bad altitude reading, ") +
                            std::string("setting climb_angle to zero. \n"));
            climb_angle = 0.0;
        } else {
            // If we get a valid altitude reading, apply the policy to translate this to a climb angle
            climb_angle = (*climb_ang_policy) ((double)range_msg.range, min_agl, max_agl);
        }
        // Run a full PID iteration
        bool wall_contact   = true;
        bool close_obstacle = false;
        follow_wall(dt, orbit_angle, climb_angle, wall_contact, close_obstacle);
        /** NOTE: this flag is never activated if obstacle detection disabled during orbiting **/
        if (close_obstacle) {
            // This means the drone has reached an unsafe distance from the obstacle
            if (get_closest_obstacle_position(obst_pos, obst_frame_id)) {
                handle_missed_wall(obst_pos, obst_frame_id, side_vel, orbit_angle);
            } else {
                m_logger.logstr(std::string("bug3_obstacle_handler: get_closest_obstacle_position ") +
                                std::string("failed to detect obstacle flagged as close.\n"));
            }
        } else if (!wall_contact) {
            /** NOTE: At this point, we simply assume the obstacle was surpassed in height.
             * If that is not the case, the drone will eventually return in obstacle-handling mode */ 
            m_logger.logstr("bug3_obstacle_handler: Obstacle surpassed in height\n");
            // We are forced to increase in height regardless of altitude constraints,
            // else the drone won't get wall contact back (PIDs will handle altitude deviations)
            while (!change_alt(min_agl, linear_speed * sin(climb_angle))) {}
            m_logger.logstr("bug3_obstacle_handler: Drone successfully raised by buffer height min_agl.\n");
            break; // exit loop since obstacle_handling procedure has now terminated
        }
        update_position_vector(curr_pos);
    }
    m_logger.logstr(std::string("bug3_obstacle_handler: Terminating obstacle handling procedure at (") +
                    std::to_string(curr_pos.getX()) + std::string(", ") + 
                    std::to_string(curr_pos.getY()) + std::string(", ") +
                    std::to_string(curr_pos.getZ()) + std::string(")\n"));
}

void VelController::vertical_vel_check(bool& alt_within_range,
                                       double& vertical_vel,
                                       const double min_agl,
                                       const double max_agl,
                                       const PIDparams& alt_pid_params)
{
    /** NOTE: the function does not manipulate the vertical velocity if it gets a bad reading **/
    range_sh->update_msg();
    sensor_msgs::Range range_msg = range_sh->currMsg();
    if (!is_within_range(range_msg)) {
        m_logger.logstr("vertical_vel_check: got bad range. returning.\n");
        return;
    }
    double curr_agl = range_msg.range;
    // Check if the altitude is within the desired range
    if ((curr_agl > min_agl) && (curr_agl < max_agl)) {
        alt_within_range = true;
        return;
    }
    // Otherwise, if the altitude just got out of range, 
    // reset the altitude PID since we are starting a new altitude correction sequence
    if (alt_within_range) {
        alt_pid->reset(alt_pid_params);
        alt_within_range = false;
    }
    // Compute the resulting error & PID output by taking into account both possible deviation cases
    // This ensures positive command when below minimum & negative command when above maximum
    double next_err = (curr_agl < min_agl) * (min_agl - curr_agl) +    
                      (curr_agl > max_agl) * (max_agl - curr_agl);
    vertical_vel    = alt_pid->step(next_err, 1.0 / loop_frequency);
}

double VelController::lin_ang_pol(double alt, double min_agl, double max_agl)
{
    return (M_PI/2.0) * (max_agl - alt) / (max_agl - min_agl);
}

bool VelController::Bug3( const std::string& goal_frame_id,
                          const tf::Vector3& goal_point, 
                          const double       min_agl,
                          const double       max_agl,
                          double (*climb_ang_policy) (double, double, double),    
                          const PIDparams& obst_pid_params,
                          const PIDparams& alt_pid_params,
                          const double dt, 
                          const double side_vel,
                          const double orbit_angle)
{
    // Retrieve odometry starting position
    tf::Vector3 start_odom;
    update_position_vector(start_odom);
    // Transform goal point to odometry frame
    if (!update_transform(odom_sh->currMsg().header.frame_id, goal_frame_id)) {
        m_logger.logstr("Cannot get goal_frame -> odometry transform.");
        return false;
    }
    tf::Vector3 goal_odom = T(goal_point);
    m_logger.logstr(std::string("goal_frame_id transformed to ") + odom_sh->currMsg().header.frame_id + 
                    std::string(" ; goal point to ( ") + std::to_string(goal_odom.getX()) + ", " +
                    std::to_string(goal_odom.getY()) + ", " + std::to_string(goal_odom.getZ()) + " )\n");
    // Rotate toward goal point
    rotateYaw(atan2(goal_odom.getY() - start_odom.getY(), goal_odom.getX() - start_odom.getX()), 
                ang_tol, angular_velocity, loop_frequency);
    // Prepare messages and loop iteration
    ros::Rate rate (loop_frequency);
    geometry_msgs::TwistStamped cmd_vel_msg;
    sensor_msgs::LaserScan      laser_msg;
    sensor_msgs::Range          range_msg;
    tf::Vector3 curr_pos;
    bool alt_within_range = true;
    // Enter the main loop
    while (ros::ok() && (absolute_value(curr_pos.getX() - goal_odom.getX()) > point_tol.getX() ||
                         absolute_value(curr_pos.getY() - goal_odom.getY()) > point_tol.getY() ||
                         absolute_value(curr_pos.getZ() - goal_odom.getZ()) > point_tol.getZ())) {
        // Initialize vertical velocity to its default value
        double xyz_d = xyz_distance(curr_pos, goal_odom);
        double xy_d  = xy_distance (curr_pos, goal_odom);
        double vertical_vel = linear_speed * (goal_odom.getZ() - curr_pos.getZ()) / xyz_d;
        /** Pass vertical velocity through a PID control to enforce the required height range
         * NOTE: this modification will alter the overall linear_speed magnitude of the drone 
        /* We can re-normalize the vector, but for heuristics we ignore this aspect for now */
        vertical_vel_check(alt_within_range, vertical_vel, min_agl, max_agl, alt_pid_params);
        // Attempt to move staright towards the goal, and call obstacle handler in case of failure
        laser_sh->update_msg();
        laser_msg = laser_sh->currMsg();
        if (!omnidirectional_obstacle_check(tf::Vector3(linear_speed * (goal_odom.getX() - curr_pos.getX()) / xy_d, 
                                                        linear_speed * (goal_odom.getY() - curr_pos.getY()) / xy_d, 
                                                        vertical_vel),
                                            tf::Vector3(0.0, 0.0, 0.0), 
                                            odom_sh->currMsg().header.frame_id, // commands relative to odom frame
                                            laser_msg, // laser message was just updated
                                            cmd_vel_msg)) 
        {
            bug3_obstacle_handler( start_odom, goal_odom, min_agl, max_agl, climb_ang_policy,
                                    obst_pid_params, dt, side_vel, orbit_angle);
            /** NOTE: we still need to handle the case where bug3_obstacle_handler fails,
             * as the algorithm is not yet complete */
            rotateYaw( atan2(goal_odom.getY() - curr_pos.getY(), goal_odom.getX() - curr_pos.getX()), 
                        ang_tol, angular_velocity, loop_frequency);
            // Pass altitude control back to altitude PID controller after avoiding the obstacle
            alt_within_range = true;
        }
        vel_ph->publish(cmd_vel_msg); // remember to publish message after obstacle check!
        rate.sleep();
        update_position_vector(curr_pos);
    }
    return true; 
}

bool VelController::test_wall_pid_params(PIDparams params,
                                         const double time_goal,
                                         const double dt, 
                                         const double orbit_angle, 
                                         const double climb_angle,
                                         bool& wall_contact,
                                         bool& close_obstacle)
{
    obst_pid->reset(params);
    double startTime = ros::Time::now().toSec();
    while (ros::ok()) {
        double currTime = ros::Time::now().toSec();
        follow_wall(dt, orbit_angle, climb_angle, wall_contact, close_obstacle);
        // Determine whether PID was successful for this iteration
        bool pid_success = wall_contact && !close_obstacle;
        if (!pid_success) {
            m_logger.logstr(std::string("test_wall_pid_params failed for ") +
                            params.toString() + " after " +
                            std::to_string( currTime - startTime) + " seconds.\n");
            return false;
        } else if (currTime - startTime > time_goal) {
            m_logger.logstr(std::string("test_wall_pid_params succeeded for ") + 
                            params.toString() + "\n");
            return true;
        }  
    }
}
   
bool VelController::ZN_tuning_test( std::string  outfile,
                                    const double Kp_start,
                                    const double Kp_end,
                                    const double Kp_step,
                                    const double time_goal,
                                    const double dt, 
                                    const double side_vel,
                                    const double orbit_angle,
                                    const double climb_angle)
{
    tf::Vector3 obst_pos;
    std::string obst_frame_id;
    bool success = false; // whether at least one pid configuration passed the test 
    m_logger.logstr("Starting ZN_tuning_test\n" );
    // Initialize proportional PID parameter
    double Kp_curr = Kp_start;
    // Run tests for all desired Kp values
    while (ros::ok() && (Kp_curr < Kp_end)) {
        sleep(1); // Sleep between tuning tests to allow updating of subscribers
        m_logger.logstr("Starting ZN_tuning_test for Kp = " + std::to_string(Kp_curr) + "\n" );
        // Test PID with the desired parameters
        bool wall_contact   = true;
        bool close_obstacle = false;
        // Check for success
        if (test_wall_pid_params(PIDparams(0.0, Kp_curr, 0.0, 0.0, -1), time_goal,
                        dt, orbit_angle, climb_angle, wall_contact, close_obstacle)) {
            success = true;
        // Then handle PID errors directly
        } else if (!wall_contact) {
            if (!handle_lost_wall_contact(obst_pos, obst_frame_id)) {
                m_logger.logstr(std::string("ZN_tuning_test: lost wall contact, terminating test.\n"));
                goto END; // exit function since this error cannot be recovered
            }
            handle_missed_wall(obst_pos, obst_frame_id, side_vel, orbit_angle);
        } else if (close_obstacle) {
            if (!get_closest_obstacle_position(obst_pos, obst_frame_id)) {
                m_logger.logstr(std::string("ZN_tuning_test: failed to track obstacle flagged as close.\n"));
                break; // pass to next parameter to test (recoverable error)
            }
            handle_missed_wall(obst_pos, obst_frame_id, side_vel, orbit_angle);
        }
        // Log PID results to file
        obst_pid->log2file(outfile);
        // Increment proportional term by desired step after each iteration
        Kp_curr += Kp_step;
    }
    m_logger.logstr("Ending ZN_tuning_test gracefully.\n" );
    END:
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
    rotateYaw((tang_ang + orbit_angle), ang_tol, angular_velocity, loop_frequency);
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
        // Move radially away/toward from the obstacle, using computed velocity
        omnidirectional_obstacle_check( tf::Vector3(dir * ovf_lv.getX(), 
                                                    dir * ovf_lv.getY(),
                                                    0.0),
                                        tf::Vector3(0.0, 0.0, 0.0),
                                    //! remember that get_wall_tangential_angle works in the odometry frame
                                        odom_sh->currMsg().header.frame_id,
                                        laser_msg,
                                        cmd_vel_msg);
        vel_ph->publish(cmd_vel_msg);
        // Update current wall distance
        curr_wd = min_wall_distance(laser_msg, wsp);
    }
    stop();
    m_logger.logstr("handle_missed_wall: Moved back to goal distance.\n" );
    // For further precision, we can update again the pose of the drone
    if (final_rot_adjustment) {
        // Try to update the obstacle position
        tf::Vector3 new_obst_pos;
        std::string new_obst_frame_id;
        if (!get_closest_obstacle_position(new_obst_pos, new_obst_frame_id)) {
            m_logger.logstr(std::string("handle_missed_wall: ") +
                            std::string("Error updating closest obstacle position. Using starting value.\n" ));
            new_obst_pos      = obst_pos;
            new_obst_frame_id = obst_frame_id;
        }
        // Rotate back to the desired orbiting angle
        rotateYaw((get_wall_tangential_angle(new_obst_pos, new_obst_frame_id) + orbit_angle),
                    ang_tol, angular_velocity, loop_frequency);
        m_logger.logstr("handle_missed_wall: Rotated back to orbiting_angle.\n" );
    }
    return true;
}

void VelController::reset_PID( const PIDparams nparams) 
{
    // Reset PID with desired values
    obst_pid->reset( nparams);
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

inline bool VelController::is_within_range(const sensor_msgs::LaserScan& laser_msg, const int i)
{
    return (laser_msg.ranges[i] > laser_msg.range_min &&
            laser_msg.ranges[i] < laser_msg.range_max);
}

inline bool VelController::is_within_range(const sensor_msgs::Range& range_msg)
{
    return (range_msg.range > range_msg.min_range &&
            range_msg.range < range_msg.max_range);
}

double VelController::min_distance(const sensor_msgs::LaserScan& laser_msg,
                                   double min_scan_angle, 
                                   double max_scan_angle,
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

bool VelController::change_alt(double alt_gain, 
                               double climb_vel,
                               double max_dur)
{
    sensor_msgs::Range range_msg;
    geometry_msgs::TwistStamped cmd_vel_msg;
    range_sh->update_msg();
    range_msg = range_sh->currMsg();
    if (!is_within_range(range_msg)) {
        m_logger.logstr("change_alt error: Bad starting altitude.\n");
        return false;
    }
    double start_alt = range_msg.range;
    double curr_alt  = start_alt;
    ros::Rate rate (loop_frequency);
    // Apply a maximum loop duration to exit in case stuck
    // Increase this parameter if higher altitudes are desired
    ros::Time startTime = ros::Time::now();
    ros::Duration loopDuration (max_dur);
    while (ros::ok() && (absolute_value(curr_alt - start_alt) < 
                         absolute_value(alt_gain))) {
        // Check for time expiration
        if (ros::Time::now() > startTime + loopDuration) {
            return false;
        }
        // Update current altitude
        range_sh->update_msg();
        range_msg = range_sh->currMsg();
        // Ignore bad readings
        if (!is_within_range(range_msg)) {
            continue;
        }
        curr_alt = range_msg.range;
        cmd_vel_msg.twist.linear.z = climb_vel;
        cmd_vel_msg.header.stamp = ros::Time::now();
        vel_ph->publish(cmd_vel_msg);
        rate.sleep();
    }
    return true;
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

