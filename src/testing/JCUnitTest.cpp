
#include "JController.h"

//** Global constants **//
const std::string node_name     = "JController";
const std::string cmd_vel_topic = "/mavros/setpoint_velocity/cmd_vel";
const std::string laser_topic   = "/scan";
const std::string odom_topic    = "/mavros/local_position/odom";

int main(int argc, char** argv)
{
    ros::init(argc, argv, ::node_name);    
    ros::NodeHandle nh;                       
    sleep(2);  
    // Initialize Service handler
    ServiceHandler* Shandler = new ::ServiceHandler(&nh); 
    // Initialize laser subscriber handler
    SubHandler <sensor_msgs::LaserScan> laser_sh (&nh, ::laser_topic);
    // Initialize odometry subscriber handler
    SubHandler <nav_msgs::Odometry> odom_sh  (&nh, ::odom_topic);
    // Initialize velocity publisher handler
    PubHandler <geometry_msgs::TwistStamped> vel_ph (&nh, ::cmd_vel_topic);
    // Initialize PID controller
    PID pid_ctr ( PIDparams(0.0, 0.0, 0.0, 0.0, -1));
    // Initialize velocity controller
    VelController vel_ctr ( &vel_ph, 
                            &laser_sh, 
                            &odom_sh, 
                            &pid_ctr,
                            1.0,     // linear_speed
                            tf::Vector3(0.5, 0.5, 0.5),    // point_tol
                            0.5,          // angular_velocity
                            0.1,          // ang_tol     
                            ScanParameters(-M_PI/2.0, M_PI/4.0, 2.0), // osp
                            ScanParameters(-M_PI,     3.0,      5.0), // wsp
                            1000 //loop_frequency
                        );
    // Finally, initialize JController
    JController* JController = new ::JController(&vel_ph, &laser_sh, &vel_ctr,
                                                 tf::Vector3(1.0f, 1.0f, 1.0f),
                                                 tf::Vector3(1.0f, 1.0f, 1.0f));
    Shandler->reqMode(MAV_MODE_PREFLIGHT, "GUIDED");
    Shandler->reqArming(true);
    Shandler->reqTakeoff(0, 0, 0, 0, 2);
    sleep(10);  // sleep to give takeoff time to execute
    JController->handleKeypress(); // enter manual flight until user presses 'q'
    Shandler->reqLand(0, 0, 0, 0, 0);
    sleep(15);  // sleep to give landing time to execute
    Shandler->reqArming(false);  // Landing command already takes care of disarming
    // Clean up
    return 0;                         
}
