
#include "JController.h"
#include "ServiceHandler.h"

JController::JController(ros::NodeHandle* nh,
    double lx = 0.5, double ly = 0.5, double lz = 0.5,
    double ax = 0.5, double ay = 0.5, double az = 0.5)
    : linearX(lx), linearY(ly), linearZ(lz), angularX(ax), angularY(ay), angularZ(az)
{
    /* Initialize publishers */
    this->cmd_vel_pub = nh->advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", ::queue_size); 
}

void 
JController::publish_velocities(double lx = 0.0, double ly = 0.0, double lz = 0.0,
                double ax = 0.0, double ay = 0.0, double az = 0.0)
{
    geometry_msgs::TwistStamped msg;
    // Update transform from base_link_orientation to odom
    try {
        listener.lookupTransform("odom", "base_link_orientation", ros::Time(0), this->blo_to_odom_transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return;
    }
    // Use updated transform to set message parameters with respect to odom
    tf::Vector3 lt = this->blo_to_odom_transform(tf::Vector3(lx, ly, lz));
    tf::Vector3 at = this->blo_to_odom_transform(tf::Vector3(ax, ay, az));
    msg.twist.linear.x  = lt.getX();
    msg.twist.linear.y  = lt.getY();
    msg.twist.linear.z  = lt.getZ();
    msg.twist.angular.x = at.getX();
    msg.twist.angular.y = at.getY();
    msg.twist.angular.z = at.getZ();
    // Set message header and publish message
    msg.header.frame_id = "odom";
    msg.header.stamp    =  ros::Time::now();
    this->cmd_vel_pub.publish(msg);
}

bool 
JController::handleCommand(char cmd)
{
    switch (cmd) {
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
        return true; // return true if we want to break out of loop
    }
    return false;
}

void 
JController::handleKeypress()
{
    char cmd;
    ros::Rate rate(::frequency);
    // Setup ncurses
    initscr();
    cbreak();
    noecho();
    timeout(::tout);
    while (ros::ok()) {
        if (handleCommand(cmd = getch())) {
            break; // break loop whenever true is returned
        }
        rate.sleep();
    }
    // Clean up ncurses
    nocbreak(); //return terminal to "cooked" mode
    echo();
    endwin();
}


/* UNIT-TESTING */
#ifdef JCUT

int main(int argc, char** argv)
{
    ros::init(argc, argv, "JController");    
    ros::NodeHandle nh;                       
    sleep(2);                            
    JController* JController = new ::JController(&nh);
    ServiceHandler* Shandler = new ::ServiceHandler(&nh); 
    Shandler->reqMode(MAV_MODE_PREFLIGHT, "GUIDED");
    Shandler->reqArming(true);
    Shandler->reqTakeoff(0, 0, 0, 0, 2);
    sleep(10);  // sleep to give takeoff time to execute
    JController->handleKeypress(); // enter manual flight until user presses 'q'
    Shandler->reqLand(0, 0, 0, 0, 0);
    sleep(15);  // sleep to give landing time to execute
    Shandler->reqArming(false);  // Landing command already takes care of disarming
    delete(JController);                              
}

#endif //JCUT


