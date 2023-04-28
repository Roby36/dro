
/*** This is a non-ROS program intended to execute on a client PC ***/
/*** All ROS-related aspects are handled by JControllerServer.cpp on the other end ***/

#include "../src/ClientSocket.cpp"
#include <curses.h>          // user keyboard input

const int frequency  = 1000; // for-loop frequency
const int tout       = 100;  // ncurses timeout 

int main(int argc, char** argv) {
    // Usage check
    if (argc != 2) {
        printf("Usage: %s <server address>\n", argv[0]);
        exit(1);
    }
    // Initialize client socket
    ClientSocket* client_sock = new ClientSocket(argv[1], ROSPORT);
    // Setup ncurses and client
    initscr();
    cbreak();
    noecho();
    timeout(::tout);
    if (client_sock->init()) {
        char buff[2] = {'\0', '\0'}; // one character suffices
        // Run until user decides to terminate with ˆC
        for(;;) {
            buff[0] = getch();
            client_sock->send(buff);
            sleep((float) 1.0f / (float) frequency);
        }
    }
    // Clean up ncurses
    nocbreak(); //return terminal to "cooked" mode
    echo();
    endwin();
    return 0;
}