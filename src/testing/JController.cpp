
#include "JController.h"

void JController::publish_checked_velocities(tf::Vector3 l_vel, tf::Vector3 a_vel)
{
    // Update subscriber
    laser_sh->update_msg();
    // Initialize velocity message
    geometry_msgs::TwistStamped cmd_vel_msg;
    // Pass velocity and laser messages to VelController module
    vel_ctr->omnidirectional_obstacle_check(l_vel, a_vel, input_vel_frame_id,
        ang_range, thresh_distance, laser_sh->currMsg(), cmd_vel_msg);
    // Stamp & publish the resulting message
    cmd_vel_msg.header.stamp = ros::Time::now();
    vel_ph->publish(cmd_vel_msg);
}

void JController::navtest()
{
    ROS_INFO_STREAM("Starting navtest");
    // Hard-coded navigation test for the VelController module
    vel_ctr->reset_PID(0.0, 0.3, 0.0, 0.4);
    while (ros::ok()) {
        vel_ctr->follow_wall(4.0, 0.0, M_PI/4.0, 3.0, -M_PI/2.0, 3.0, 
            0.5, 0.5, 0.1, 100);
    }
    ROS_INFO_STREAM("Ending navtest");
}

void JController::bug2test()
{
    ROS_INFO_STREAM("Starting bug2test");
    vel_ctr->Bug2("map",
                  tf::Vector3(20.0, 20.0, 20.0),
                  tf::Vector3(1.0, 1.0, 1.0),
                  tf::Vector3(0.5, 0.0, 0.0),
                  tf::Vector3(0.0, 0.0, 0.5),
                  tf::Vector3(0.1, 0.1, 0.1),
                  tf::Vector3(1.0, 1.0, 1.0),
                  M_PI/8.0,
                  3.0,
                  100);
    ROS_INFO_STREAM("Ending bug2test");
}

bool 
JController::handleCommand(char cmd)
{
    // Initialize linear & angular components at zero
    tf::Vector3 l_vel (0, 0, 0);
    tf::Vector3 a_vel (0, 0, 0);
    switch (cmd) {
        // Assign linear & angular components based on incoming command
        case Keypress::UP:        l_vel.setZ(  lv.getZ()); break;
        case Keypress::DOWN:      l_vel.setZ( -lv.getZ()); break;
        case Keypress::CW:        a_vel.setZ( -av.getZ()); break;
        case Keypress::CCW:       a_vel.setZ(  av.getZ()); break;
        case Keypress::FORWARDS:  l_vel.setX(  lv.getX()); break;
        case Keypress::BACKWARDS: l_vel.setX( -lv.getX()); break;
        case Keypress::RIGHT:     l_vel.setY( -lv.getY()); break;
        case Keypress::LEFT:      l_vel.setY(  lv.getY()); break;
        case Keypress::NAVTEST:   navtest();               break;
        case Keypress::BUG2TEST:  bug2test();              break;
        case Keypress::QUIT:      return true;
        default:                                           break;
    }
    publish_checked_velocities(l_vel, a_vel);
    return false;
}

void 
JController::handleKeypress()
{
    char cmd;
    ros::Rate rate(frequency);
    // Setup ncurses
    initscr();
    cbreak();
    noecho();
    timeout(tout);
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

