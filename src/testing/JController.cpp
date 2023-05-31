
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

bool 
JController::handleCommand(char cmd)
{
    // Initialize linear & angular components at zero
    tf::Vector3 l_vel (0, 0, 0);
    tf::Vector3 a_vel (0, 0, 0);
    switch (cmd) {
        // Movement commands
        case Keypress::UP:        l_vel.setZ(  lv.getZ()); break;
        case Keypress::DOWN:      l_vel.setZ( -lv.getZ()); break;
        case Keypress::CW:        a_vel.setZ( -av.getZ()); break;
        case Keypress::CCW:       a_vel.setZ(  av.getZ()); break;
        case Keypress::FORWARDS:  l_vel.setX(  lv.getX()); break;
        case Keypress::BACKWARDS: l_vel.setX( -lv.getX()); break;
        case Keypress::RIGHT:     l_vel.setY( -lv.getY()); break;
        case Keypress::LEFT:      l_vel.setY(  lv.getY()); break;
        // Navigation tests
        case Keypress::NAVTEST:   navtest();               break;
        case Keypress::BUG2TEST:  bug2test();              break;
        case Keypress::ZNTEST:    ZNtest();                break;
        // Reproduce bugs by hard-coding internal VelController functions
        case Keypress::ROTATE:    rotateYaw(3.73,
                                            0.1,
                                            0.5,
                                            100
                                           );              break;
        // Quitting command
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


//!HARD-CODED NAVIGATION TESTS

void JController::navtest()
{
    /* Function needs updating

    ROS_INFO_STREAM("Starting navtest");
    // Hard-coded navigation test for the VelController module
    vel_ctr->reset_PID( PIDparams(0.0, 0.3, 0.0, 0.4, -1));
    while (ros::ok()) {
        vel_ctr->follow_wall(ScanParameters(0.0, M_PI/4.0, 4.0),  
                             ScanParameters(-M_PI/2.0, 3.0, 3.0),
                             tf::Vector3(0.5, 0.0, 0.0),
                             tf::Vector3(0.0, 0.0, 0.5),
                             0.1,
                             100
                            );
    }
    ROS_INFO_STREAM("Ending navtest");

    */
}

void JController::bug2test()
{
    /* Function needs updating

    ROS_INFO_STREAM("Starting bug2test");
    vel_ctr->Bug2("map", // goal_frame_id
                  tf::Vector3(20.0, 20.0, 20.0), // goal_point
                  tf::Vector3(1.0, 1.0, 1.0), // goal_tol
                  tf::Vector3(0.5, 0.0, 0.0), // linear_velocity
                  0.5, // angular_velocity
                  0.2, // ang_tol
                  tf::Vector3(1.0, 1.0, 1.0), // line_tol
                  PIDparams(0.0, 0.2, 0.0, 0.2, -1), // pid_params
                  ScanParameters(0.0, M_PI/4.0, 4.0), // osp
                  ScanParameters(-M_PI/2.0, 3.0, 3.0), // wsp
                  tf::Vector3(0.5, 0.0, 0.0), // lv
                  tf::Vector3(0.0, 0.0, 0.5), // av
                  0.1, // dt
                  100, // loop_frequency
                  M_PI/8.0, // obst_ang_range
                  3.0, // obst_thresh_distance
                  100 // frequency
                  ); 
    ROS_INFO_STREAM("Ending bug2test");

    */
}

void JController::ZNtest()
{
    vel_ctr-> ZN_tuning_test("ZN_test.txt",
                            0.022,
                            0.05,
                            0.002,
                            600.00,
                            //! Parameters for follow_wall()
                            ScanParameters(0.0,       M_PI/8.0, 1.1), // obstacle (THRESH > RANGE_MIN!)
                            ScanParameters(-M_PI/2.0, 3.0, 5.0), // wall
                            tf::Vector3(0.5, 0.0, 0.0), // linear velocity
                            0.1,  // dt
                            100, // loop_frequency
                            //! Parameters for rotating drone after PID failure
                            0.2, //ang_tol
                            0.2, //ang_vel 
                            100,          // rot_frequency
                            //! Parameters for drone translation after PID failure
                            0.2,  // lin_vel
                            1.0,  // wd_tol
                            100   // freq
                            ); 
}

void JController::rotateYaw(const double input_yaw, 
                            const double tol, 
                            const double ang_vel,  
                            int frequency)
{
    vel_ctr->rotateYaw(input_yaw, tol, ang_vel, frequency);
}

//!HARD-CODED NAVIGATION TESTS
