
/*** This is a non-ROS program intended to execute on a client PC ***/
/*** All ROS-related aspects are handled by JControllerServer.cpp on the other end ***/

#include "UDPclient.h"
#include <curses.h>          // user keyboard input

const int buffSize   = MAXTIMEDIGITS + 2; // size of buffer to be sent
const int frequency  = 1000;            // for-loop frequency
const int tout       = 100;           // ncurses timeout 

int main(int argc, char** argv) {
    // Usage check
    if (argc != 2) {
        printf("Usage: %s <server address>\n", argv[0]);
        exit(1);
    }
    // Initialize client socket, logging to a dedicated file
    FILE* log_fp = fopen("./client_log.txt", "w");
    if (log_fp == NULL) {
        fprintf(stderr, "Error opening log file.\n");
        exit(1);
    }
    UDPclient* client_sock = new UDPclient(argv[1], CMDVELPORT, log_fp);
    // Setup ncurses and client
    initscr();
    cbreak();
    noecho();
    timeout(::tout);
    if (client_sock->init()) {
        char buff[buffSize]; // initialize buffer
        // Run until user decides to terminate with ˆC
        for(;;) {
            // IMPORTANT: Truncate buffer each time to reset it fully
            buff[0] = '\0';
            snprintf(buff, 10, "%c", getch());
            // buff[0] = getch();
            client_sock->send_timestamped(buff);
            sleep((float) 1.0f / (float) frequency);
        }
    }
    // Clean up ncurses
    nocbreak(); //return terminal to "cooked" mode
    echo();
    endwin();
    // Close logging file
    fclose(log_fp);
    return 0;
}