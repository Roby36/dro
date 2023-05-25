
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
                                                   const std::string& output_vel_frame_id,
                                                   const std::string& input_laser_frame_id,
                                                   const double ang_range,
                                                   const double thresh_distance,
                                                   const sensor_msgs::LaserScan& laser_msg,
                                                    geometry_msgs::TwistStamped& cmd_vel_msg)
{
    // First transform input velocity to working frame for the module
    if (!update_transform(this->working_frame_id, input_vel_frame_id)) {
        ROS_INFO_STREAM("Cannot get input_vel_frame -> working_frame transform.");
        return false;
    }
    tf::Vector3 wf_lv = T(lv);
    tf::Vector3 wf_av = T(av); // av currently ignored
    // Identify requested direction of movement wrt working frame
    double dir_ang = atan2(wf_lv.getY(), wf_lv.getX());
    int min_index = 0, max_index = 0;
    ranges_indices(laser_msg, dir_ang - (ang_range / 2.0), dir_ang + (ang_range / 2.0),
                    min_index, max_index);
    // Iterate through all values in array
    for (int i = min_index; i < max_index; i++) {
        // If one single obstacle detected within threshhold distance, stop drone
        if (laser_msg.ranges[i] < thresh_distance &&
            laser_msg.ranges[i] > laser_msg.range_min &&
            laser_msg.ranges[i] < laser_msg.range_max) {
            // If obstacle too close, return false without publishing velocities
            ROS_INFO_STREAM( "Stopping: obstacle detected within distance "
                            << thresh_distance << " at laser scan angle "
                            << i*laser_msg.angle_increment + laser_msg.angle_min);
            return false;
        }
    }
    // If no obstacle is detected, forward input command to cmd_vel_msg in desired frame
    if (!update_transform(output_vel_frame_id, this->working_frame_id)) {
        ROS_INFO_STREAM("Cannot get working_frame_id -> output_vel_frame_id transform.");
        return false;
    }
    tf::Vector3 ovf_lv = T(wf_lv);
    tf::Vector3 ovf_av = T(wf_av); // av currently ignored
    // Set geometry_msg vectors to tf vectors
    vector3TFToMsg(ovf_lv, cmd_vel_msg.twist.linear);
    vector3TFToMsg(ovf_av, cmd_vel_msg.twist.angular);
    // Set message header (can be set before function call)?
    cmd_vel_msg.header.frame_id = output_vel_frame_id;
    cmd_vel_msg.header.stamp    = ros::Time::now();
    return true;
}
