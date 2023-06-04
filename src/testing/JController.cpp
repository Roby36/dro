
#include "JController.h"

void JController::publish_checked_velocities(tf::Vector3 l_vel, tf::Vector3 a_vel)
{
    // Update subscriber
    laser_sh->update_msg();
    // Initialize velocity message
    geometry_msgs::TwistStamped cmd_vel_msg;
    // Pass velocity and laser messages to VelController module
    vel_ctr->omnidirectional_obstacle_check(l_vel, a_vel, input_vel_frame_id,
                                            laser_sh->currMsg(), cmd_vel_msg);
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
        /** TODO: service commands */

        // Movement commands (wrt input_vel_frame_id)
        case Keypress::UP:        l_vel.setZ(  lv.getZ()); break;
        case Keypress::DOWN:      l_vel.setZ( -lv.getZ()); break;
        case Keypress::CW:        a_vel.setZ( -av.getZ()); break;
        case Keypress::CCW:       a_vel.setZ(  av.getZ()); break;
        case Keypress::FORWARDS:  l_vel.setX(  lv.getX()); break;
        case Keypress::BACKWARDS: l_vel.setX( -lv.getX()); break;
        case Keypress::RIGHT:     l_vel.setY( -lv.getY()); break;
        case Keypress::LEFT:      l_vel.setY(  lv.getY()); break;
        // Navigation tests
        case Keypress::NAVTEST:   navtest2();              break;
        case Keypress::BUG3TEST:  bug3test();              break;
        case Keypress::ZNTEST:    ZNtest();                break;
        case Keypress::TWISTTEST: rotate_in_line(tf::Vector3(-0.1, -0.1, 0.1),
                    "/odom", 0.1, 10.0);                  break;
        // Reproduce bugs here by hard-coding internal VelController functions
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
    /** IMPORTANT:
     * These specific PID constants are tuned for the following VelController parameters
     * (initialized in JCUnitTest module), and require the Gazebo world "PID_test_cyl.world"
     * 
        VelController vel_ctr ( &vel_ph, 
                                &laser_sh, 
                                &odom_sh, 
                                &pid_ctr,
                                tf::Vector3(0.5, 0.0, 0.0),    // linear_velocity
                                tf::Vector3(0.5, 0.5, 0.5),    // point_tol
                                0.5,          // angular_velocity
                                0.1,          // ang_tol     
                                ScanParameters(0.0,       M_PI/2.0, 2.0), // osp
                                ScanParameters(-M_PI/2.0, 3.0,      8.0), // wsp
                                100 //loop_frequency
                            );
    */

   ROS_INFO_STREAM("Starting navtest");
    // Hard-coded navigation test for the VelController module (using ZN tuned parameters)
    vel_ctr->reset_PID( PIDparams(0.0, 0.0072, 0.0002, 0.0675, -1));
    while (ros::ok()) {
        vel_ctr->follow_wall(0.1, 0.0, 0.0);
    }
    ROS_INFO_STREAM("Ending navtest");
}

void JController::navtest2(double dur)
{
    /* Before running this test, make sure vel_ctr is initialized appropriately in JCUnitTest.cpp */

    ROS_INFO_STREAM("Starting navtest2");
    // Reset PID to appropriate values
    vel_ctr->reset_PID( PIDparams(0.0, 0.1, 0.0, 0.1, -1));
    // Declare all function paramters
    tf::Vector3 obst_pos;
    std::string obst_frame_id;
    double dt          = 1.0;
    double side_vel    = 0.5;
    double orbit_angle = M_PI/4.0;
    double climb_angle = 0.0;
    // Time loop duration
    ros::Time startTime = ros::Time::now();
    ros::Duration loopDuration (dur);
    while (ros::ok() && (ros::Time::now() < startTime + loopDuration)) {
        // Flag variables for error handling (reset on each loop iteration)
        bool wall_contact   = true;
        bool close_obstacle = false;
        vel_ctr->follow_wall(dt, orbit_angle, climb_angle,
                            wall_contact, close_obstacle);
        // Handle errors (from OUTSIDE follow_wall!)
        if (!wall_contact) {
            vel_ctr->handle_lost_wall_contact(obst_pos, obst_frame_id);
            vel_ctr->handle_missed_wall(obst_pos, obst_frame_id, side_vel, orbit_angle);
        } else if (close_obstacle) {
            vel_ctr->get_closest_obstacle_position(obst_pos, obst_frame_id);
            vel_ctr->handle_missed_wall(obst_pos, obst_frame_id, side_vel, orbit_angle);
        }
    }
    ROS_INFO_STREAM("Ending navtest2");
}

void JController::bug3test()
{
    ROS_INFO_STREAM("Starting bug3test");
    vel_ctr->Bug3( "/map",
                    tf::Vector3(20.0, 20.0, 20.0), 
                //! Altitude ABOVE GROUND bounds
                    2.0,
                    10.0,
                //! Function determining climb angle when travelling around obstacle
                    vel_ctr->lin_ang_pol,  
                //! Parameters for PID
                    PIDparams(0.0, 0.1, 0.0, 0.1, -1),
                    PIDparams(0.0, 0.1, 0.0, 0.1, -1),
                //! Parameters for follow_wall phase
                    1.0, 
                    0.2,
                    0.0);
    ROS_INFO_STREAM("Ending bug3test");

}

void JController::ZNtest()
{
    /* Function needs updating
    vel_ctr-> ZN_tuning_test("ZN_test.txt",
                            0.01,
                            0.10,
                            0.002,
                            600.00,
                        //! Parameters for follow_wall()
                            0.1,  // dt
                            0.2  // side_vel
                            ); 
    */
}

void JController::rotate_in_line( const tf::Vector3& linear_vel,
                                  const std::string& input_vel_frame_id,
                                  const double ang_vel,
                                  const double exec_time)
{
    vel_ctr->rotate_in_line(linear_vel, input_vel_frame_id, ang_vel, exec_time);
}

void JController::rotateYaw(const double input_yaw, 
                            const double tol, 
                            const double ang_vel,  
                            int frequency)
{
    vel_ctr->rotateYaw(input_yaw, tol, ang_vel, frequency);
}

//!HARD-CODED NAVIGATION TESTS
