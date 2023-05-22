
#include "TransformBroadcaster2.h"

void 
TransformBroadcaster::broadcast_transform(const tf::Vector3&    position,
                                          const tf::Quaternion& orientation,
                                          const std::string&    parent_id, 
                                          const std::string&    child_id,
                                                tf::Transform&  transform)
{
    transform.setOrigin(position);
    transform.setRotation(orientation);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_id, child_id));
}

void 
TransformBroadcaster::broadcast_moving_frames()
{
    // Update odometry message & retrieve most current one
    do {
        ros::spinOnce();
    } while (!odom_sh->is_initialized());
    const nav_msgs::Odometry msg = odom_sh->currMsg();
    // Convert message point to tf::Vector3
    tf::Vector3 bt_v;
    pointMsgToTF(msg.pose.pose.position, bt_v);
    // Convert message quaternion to tf::Quaternion
    tf::Quaternion bt;
    quaternionMsgToTF(msg.pose.pose.orientation, bt);
    // Construct & send each moving transform (from odom)
    broadcast_transform(bt_v, bt, "odom", "base_link", this->odomTbl);
    broadcast_transform(tf::Vector3(0.0f, 0.0f, 0.0f), bt, 
                        "odom", "base_link_orientation", this->odomTblo);
}

void TransformBroadcaster::broadcast_static_frames()
{
    // map->odom transform is set here
    // For now we assume map and odom correspond
    tf::Quaternion q;
    q.setRPY(0, 0, 0); 
    q.normalize();
    broadcast_transform(tf::Vector3(0.0f, 0.0f, 0.0f), q, "/map", "/odom", this->mapTodom);
    /* IMPORTANT:
    /* Base_scan static transform set-up using sensor's settings in
    /* /ardupilot_gazebo/models/iris_with_standoffs/models.sdf */
    q.setRPY(0, 0, 0); 
    q.normalize();
    broadcast_transform(tf::Vector3(0.0f, 0.0f, 0.2f), q, "base_link", "base_scan", this->blTbs);   
}

void TransformBroadcaster::run()
{
    ros::Rate rate(this->frequency);
    while (ros::ok()) {
        broadcast_moving_frames();
        broadcast_static_frames();
        rate.sleep();
    }
}

