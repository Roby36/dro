
#include "VelController.h"

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

bool VelController::is_within_range(const sensor_msgs::LaserScan& laser_msg, const int i)
{
    return (laser_msg.ranges[i] > laser_msg.range_min &&
            laser_msg.ranges[i] < laser_msg.range_max);
}

double VelController::min_distance(const sensor_msgs::LaserScan& laser_msg,
                                   const double min_scan_angle, 
                                   const double max_scan_angle)
{
    // First retrieve min & man indexes in ranges[]
    int min_index, max_index;
    ranges_indices(laser_msg, min_scan_angle, max_scan_angle, min_index, max_index);
    // Iterate through all laser readings to find lowest distance reading
    double min_distance = laser_msg.range_max;
    for (int i = min_index; i < max_index; i++) {
        if (laser_msg.ranges[i] < min_distance && is_within_range(laser_msg, i)) {
            min_distance = laser_msg.ranges[i];
        }
    }
    return min_distance;
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

bool VelController::omnidirectional_obstacle_check(const tf::Vector3& lv, 
                                                   const tf::Vector3& av,
                                                   const std::string& input_vel_frame_id,
                                                   const double ang_range,
                                                   const double thresh_distance,
                                                   const sensor_msgs::LaserScan& laser_msg,
                                                    geometry_msgs::TwistStamped& cmd_vel_msg)
{
    //! This transformation still needs work
    //! For now we trivially assume that input_vel_frame_id = working_frame_id
    if (!update_transform(this->working_frame_id, input_vel_frame_id)) {
        ROS_INFO_STREAM("Cannot get input_vel_frame -> working_frame transform.");
        return false;
    }
    //!
    tf::Vector3 wf_lv = T(lv);
    tf::Vector3 wf_av = T(av); 
    // Now transform from working frame to output frame (i.e. "odom" for mavros)
    if (!update_transform(output_vel_frame_id, this->working_frame_id)) {
        ROS_INFO_STREAM("Cannot get working_frame_id -> output_vel_frame_id transform.");
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
    ranges_indices(laser_msg, dir_ang - (ang_range / 2.0), dir_ang + (ang_range / 2.0),
                    min_index, max_index);
    // Iterate through all values in array
    for (int i = min_index; i < max_index; i++) {
        // If one single obstacle detected within threshhold distance, stop drone
        if (laser_msg.ranges[i] < thresh_distance && is_within_range(laser_msg, i)) {
            // If obstacle too close, return false without publishing velocities
            ROS_INFO_STREAM( "Stopping: obstacle detected within distance "
                            << thresh_distance << " at laser scan angle "
                            << i*laser_msg.angle_increment + laser_msg.angle_min);
            return false;
        }
    }
    // If no obstacle is detected, fill up also linear part of message
    vector3TFToMsg(ovf_lv, cmd_vel_msg.twist.linear);
    return true;
}

void VelController::reset_PID(const double K,
                              const double Kp,
                              const double Ki,
                              const double Kd,
                              const int pid_error_sum_terms) 
{
    // Reset PID with desired values
    pid_ctr->reset(K, Kp, Ki, Kd, pid_error_sum_terms);
}

void VelController::follow_wall(const double obst_thresh_distance,
                                const double obst_scan_angle,
                                const double obst_ang_range,
                                const double wall_goal_distance,
                                const double wall_scan_angle,
                                const double wall_ang_range,
                                const double linear_velocity,
                                const double angular_velocity,
                                const double dt, // PID controller parameters:
                                const int    loop_frequency)
{
    // Update laser's message
    laser_sh->update_msg();
    // Compute the desired scanning boundary angles
    const double min_scan_angle = wall_scan_angle - (wall_ang_range / 2.0);
    const double max_scan_angle = wall_scan_angle + (wall_ang_range / 2.0);
    const double obst_min_angle = obst_scan_angle - (obst_ang_range / 2.0);
    const double obst_max_angle = obst_scan_angle + (obst_ang_range / 2.0);
    // Compute current minimum wall distance
    double curr_wall_distance = min_distance(laser_sh->currMsg(), min_scan_angle, max_scan_angle);
    // Compute resulting error
    double next_error = wall_goal_distance - curr_wall_distance;
    // Feed error in PID controller, and prepare velocity components and message
    tf::Vector3 lv (linear_velocity, 0.0, 0.0);
    tf::Vector3 av (0.0, 0.0, pid_ctr->step(next_error, dt));
    geometry_msgs::TwistStamped cmd_vel_msg;
    bool obstacle = false; // keep track of potential obstacles
    ros::Time startTime = ros::Time::now();
    ros::Duration loopDuration (dt); // dt seconds 
    ros::Rate rate(loop_frequency);  // set up frequency to control loop iteration
    // Let the drone move throughout the given time step, or rotate only if obstacle
    while (ros::ok() && ((ros::Time::now() < startTime + loopDuration) || obstacle)) {
        // If obstacle found, modify angular velocity to desired value
        if (obstacle) {
            av.setZ(angular_velocity);
        }
        // Filter message through the obstacle-checking function
        // Robot will keep rotating for entire loop as soon as obstacle detected
        obstacle = !omnidirectional_obstacle_check(lv, av, working_frame_id, 
                        obst_ang_range, obst_thresh_distance, laser_sh->currMsg(), cmd_vel_msg);
        // Stamp & publish the resulting message
        cmd_vel_msg.header.stamp = ros::Time::now();
        vel_ph->publish(cmd_vel_msg);
        // Update laser scan at each iteration
        laser_sh->update_msg();
        rate.sleep(); 
    }
}

