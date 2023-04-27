
#include "ServerSocket.cpp"

int main() {
    ServerSocket* serv_sock = new ServerSocket();
    if (serv_sock->init()) {
        serv_sock->msg_loop();
    }
    return 0;
}