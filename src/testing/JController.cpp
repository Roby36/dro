
#include "JController.h"

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
        default:                                           break;
    }
    // Publish resulting linear & angular components
    vel_ph->forward_obstacle_check(l_vel, a_vel,
                                   min_scan_angle, max_scan_angle,
                                   min_distance);
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

