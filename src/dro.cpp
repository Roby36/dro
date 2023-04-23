
#include <ros/ros.h> 
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <string> 
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <cstdint>  // for integer types
#include <curses.h> // keyboard input

const int frequency  = 1000;                                 
const int queue_size = 10;

enum BaseMode : uint8_t {
    MAV_MODE_PREFLIGHT          = 0,
    MAV_MODE_STABILIZE_DISARMED = 80,
    MAV_MODE_STABILIZE_ARMED    = 208,
    MAV_MODE_MANUAL_DISARMED    = 64,
    MAV_MODE_MANUAL_ARMED       = 192,
    MAV_MODE_GUIDED_DISARMED    = 88,
    MAV_MODE_GUIDED_ARMED       = 216,
    MAV_MODE_AUTO_DISARMED      = 92,
    MAV_MODE_AUTO_ARMED         = 220,
    MAV_MODE_TEST_DISARMED      = 66,
    MAV_MODE_TEST_ARMED         = 194
};

enum Keypress {
    UP        = 'e',
    DOWN      = 'x',
    CW        = 'd',
    CCW       = 's',
    FORWARDS  = 'i',
    BACKWARDS = 'm',
    RIGHT     = 'k',
    LEFT      = 'j', 
    QUIT      = 'q'
};

class Dro
{
    /* PUBLISHERS */
    ros::Publisher cmd_vel_pub;

    /* SERVICE CLIENTS */
    ros::ServiceClient mode_srv;
    ros::ServiceClient arming_srv;
    ros::ServiceClient takeoff_srv;
    ros::ServiceClient land_srv;

    /* TRANSFORMER & TRANSFORMS */
    tf::Transformer      my_transformer;
    tf::StampedTransform bl_to_odom_transform; 

    /* DRONE DEFAULT VELOCITIES */
    const double linearX;
    const double linearY;
    const double linearZ;
    const double angularX;
    const double angularY;
    const double angularZ;  

    public:
    /* Constructor */
    Dro(ros::NodeHandle* nh,
        double lx = 0.5, double ly = 0.5, double lz = 0.5,
        double ax = 0.5, double ay = 0.5, double az = 0.5)
        : linearX(lx), linearY(ly), linearZ(lz), angularX(ax), angularY(ay), angularZ(az)
    {
        /* Initialize publishers */
        this->cmd_vel_pub = nh->advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", ::queue_size);

        /* Initialize service clients */
        this->arming_srv  = nh->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        this->takeoff_srv = nh->serviceClient<mavros_msgs::CommandTOL> ("/mavros/cmd/takeoff");
        this->land_srv    = nh->serviceClient<mavros_msgs::CommandTOL> ("/mavros/cmd/land");
        this->mode_srv    = nh->serviceClient<mavros_msgs::SetMode>    ("/mavros/set_mode");
    }

    /* Method to publish linear and angular velocities */
    void 
    publish_velocities(double lx = 0.0, double ly = 0.0, double lz = 0.0,
                       double ax = 0.0, double ay = 0.0, double az = 0.0)
    {
        geometry_msgs::TwistStamped msg;
            /*
            // Update transform from base_link to odom
            my_transformer.lookupTransform("odom", "base_link", ros::Time::now(), this->bl_to_odom_transform);
            // Use updated transform to set message parameters with respect to odom
            tf::Vector3 lt = this->bl_to_odom_transform(tf::Vector3(lx, ly, lz));
            tf::Vector3 at = this->bl_to_odom_transform(tf::Vector3(ax, ay, az));
            */
        msg.twist.linear.x  = lx;   // lt.getX();
        msg.twist.linear.y  = ly;   // lt.getY();
        msg.twist.linear.z  = lz;   // lt.getZ();
        msg.twist.angular.x = ax;   // at.getX();
        msg.twist.angular.y = ay;   // at.getY();
        msg.twist.angular.z = az;   // at.getZ();
        // Set message header and publish message
        msg.header.frame_id = "base_link";
        msg.header.stamp    =  ros::Time::now();
        this->cmd_vel_pub.publish(msg);
    }

    /* SERVICE REQUESTS METHODS */

    bool 
    reqMode(BaseMode base_mode = MAV_MODE_PREFLIGHT, 
       std::string custom_mode = "")
    {
        mavros_msgs::SetMode::Request  req;
        mavros_msgs::SetMode::Response resp;
        req.base_mode   = base_mode;
        req.custom_mode = custom_mode;

        bool success = this->mode_srv.call(req, resp);
        if (!resp.mode_sent) {
            ROS_ERROR_STREAM("Failed to send base mode "  << base_mode << "; "  << "custom mode " << custom_mode);
        } else if (!success) {
            ROS_ERROR_STREAM("Failed to set base mode "   << base_mode << "; "  << "custom mode " << custom_mode);
        } else {
            ROS_INFO_STREAM("Successfully set base mode " << base_mode << "; "  << "custom mode " << custom_mode);
        }
    }

    bool 
    reqArming(bool value) 
    {
        mavros_msgs::CommandBool::Request  req;
        mavros_msgs::CommandBool::Response resp;
        req.value = value;

        bool success = this->arming_srv.call(req, resp);
        if (resp.success) {
            ROS_INFO_STREAM("Arming status successfully changed: result: " << resp.result);
        } else {
            ROS_ERROR_STREAM("Failed to change arming status: result: "    << resp.result);
        }
        return resp.success;
    }

    bool 
    reqTakeoff(float min_pitch, float yaw, float latitude, float longitude, float altitude)
    {
        mavros_msgs::CommandTOL::Request  req;
        mavros_msgs::CommandTOL::Response resp;
        req.min_pitch = min_pitch;
        req.yaw       = yaw;
        req.latitude  = latitude;
        req.longitude = longitude;
        req.altitude  = altitude;

        bool success = this->takeoff_srv.call(req, resp);
        if (resp.success) {
            ROS_INFO_STREAM("Takeoff successful: result: " << resp.result);
        } else {
            ROS_ERROR_STREAM("Failed to takeoff: result: " << resp.result);
        }
        return resp.success;
    }

    bool 
    reqLand(float min_pitch, float yaw, float latitude, float longitude, float altitude)
    {
        mavros_msgs::CommandTOL::Request  req;
        mavros_msgs::CommandTOL::Response resp;
        req.min_pitch = min_pitch;
        req.yaw       = yaw;
        req.latitude  = latitude;
        req.longitude = longitude;
        req.altitude  = altitude;
        
        bool success = this->land_srv.call(req, resp);
        if (resp.success) {
            ROS_INFO_STREAM("Landing successful: result: " << resp.result);
        } else {
            ROS_ERROR_STREAM("Failed to land: result: "    << resp.result);
        }
        return resp.success;
    }


    /* Method to handle keyboard input */
    void 
    handleKeypress(int timeout)
    {
        char cmd;
        ros::Rate rate(::frequency);
        // Setup ncurses
        initscr();
        cbreak();
        noecho();
        timeout(timeout);
        while (ros::ok()) {
            switch (cmd = getch()) {
                case Keypress::UP:        publish_velocities(0, 0,  this->linearZ, 0, 0, 0);  break;
                case Keypress::DOWN:      publish_velocities(0, 0, -this->linearZ, 0, 0, 0);  break;
                case Keypress::CW:        publish_velocities(0, 0, 0, 0, 0, -this->angularZ); break;
                case Keypress::CCW:       publish_velocities(0, 0, 0, 0, 0,  this->angularZ); break;
                case Keypress::FORWARDS:  publish_velocities( this->linearX, 0, 0, 0, 0, 0);  break;
                case Keypress::BACKWARDS: publish_velocities(-this->linearX, 0, 0, 0, 0, 0);  break;
                case Keypress::RIGHT:     publish_velocities(0, -this->linearY, 0, 0, 0, 0);  break;
                case Keypress::LEFT:      publish_velocities(0,  this->linearY, 0, 0, 0, 0);  break;
                default:                  publish_velocities(0, 0, 0, 0, 0, 0);               break;
            }
            if (cmd == Keypress::QUIT) {
                break;
            }
            rate.sleep();
        }
        // Clean up ncurses
        nocbreak(); //return terminal to "cooked" mode
        echo();
        endwin();
    }

};

/* MAIN */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "Drodemo");    
    ros::NodeHandle nh;                       
    sleep(2);                            
    Dro* Dro = new ::Dro(&nh); 

    Dro->reqMode(MAV_MODE_PREFLIGHT, "GUIDED");
    Dro->reqArming(true);
    Dro->reqTakeoff(0, 0, 0, 0, 2);
    sleep(10);  // sleep to give takeoff time to execute

    Dro->handleKeypress(100); // enter manual flight until user presses 'q'

    Dro->reqLand(0, 0, 0, 0, 0);
    sleep(30);  // sleep to give landing time to execute
    Dro->reqArming(false);  // Llanding command already takes care of disarming

    delete(Dro);                              
}




