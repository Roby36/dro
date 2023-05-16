
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class blBroadcaster
{
    tf::TransformBroadcaster br;
    tf::Transform blo_transform;
    tf::Transform bl_transform;
    ros::Subscriber odom_sub;
    const std::string default_odom_topic = "/mavros/local_position/odom";
    const int queue_size = 10;

    void odomCallback(const nav_msgs::Odometry& msg)
    {
        // Construct & send bl_transform
        this->bl_transform.setOrigin(tf::Vector3(msg.pose.pose.position.x, 
                                                 msg.pose.pose.position.y, 
                                                 msg.pose.pose.position.z));
        this->bl_transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x,
                                                   msg.pose.pose.orientation.y,
                                                   msg.pose.pose.orientation.z,
                                                   msg.pose.pose.orientation.w ));
        this->br.sendTransform(tf::StampedTransform(this->bl_transform, ros::Time::now(), "odom", "base_link"));
        // Construct & send blo_transform
        this->blo_transform.setOrigin(tf::Vector3(0, 0, 0));
        this->blo_transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x,
                                                   msg.pose.pose.orientation.y,
                                                   msg.pose.pose.orientation.z,
                                                   msg.pose.pose.orientation.w ));
        this->br.sendTransform(tf::StampedTransform(this->blo_transform, ros::Time::now(), "odom", "base_link_orientation"));
    }

    public:

    blBroadcaster(ros::NodeHandle* nh)
    {
        this->odom_sub = nh->subscribe(this->default_odom_topic, this->queue_size, &blBroadcaster::odomCallback, this);
    }


};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "bl_broadcaster");
    ros::NodeHandle nh;
    blBroadcaster* bc = new blBroadcaster(&nh);
    ros::spin();
    return 0;
}