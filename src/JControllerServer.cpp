
#include "ServerSocket.cpp"
#include "JController.cpp"

#ifndef SOCKSERV

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "JControllerServer");    
    ros::NodeHandle nh;                       
    sleep(2);
    // Initialize ServerSocket connection, with dedicated log file
    FILE* log_fp = fopen("w", "./server_log.txt");
    if (log_fp == NULL) {
        fprintf(stderr, "Error opening log file.\n");
        exit(1);
    }
    ServerSocket* serv_sock = new ServerSocket(ROSPORT, 2 + MAXTIMEDIGITS, log_fp);
    if (serv_sock->init()) {
        ROS_INFO_STREAM("Waiting on port " << serv_sock->port);
    }
    // Initialize controller instance and take off
    JController* JController = new ::JController(&nh); 
    JController->reqMode(MAV_MODE_PREFLIGHT, "GUIDED");
    JController->reqArming(true);
    JController->reqTakeoff(0, 0, 0, 0, 2);
    sleep(10);  // sleep to give takeoff time to execute
    // Enter infinite while loop, waiting for commands
    ros::Rate rate(::frequency);
    char cmd;
    while (ros::ok()) {
        // Extract only first character from any incoming message
        // Logging file shows full message and resulting delays
        cmd = *serv_sock->msg_wait();
        ROS_INFO_STREAM("Got command " << cmd);
        if(JController->handleCommand(cmd)) {
            break;
        }
        rate.sleep();
    }
    // Land and disarm
    JController->reqLand(0, 0, 0, 0, 0);
    sleep(15);  // sleep to give landing time to execute
    JController->reqArming(false);  // Llanding command already takes care of disarming
    // Close socket logging file
    fclose(log_fp);
    // Clean up
    delete(JController);
    return 0;                              
}

#endif // SOCKSERV
