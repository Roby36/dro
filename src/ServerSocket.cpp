
#include "DSocket.h"

class ServerSocket : public DSocket
{
    char buffer[MAXLINE];
    struct sockaddr_in client_addr;

    public:

    ServerSocket(int port = SERVERPORT, FILE* lfp = stdout) 
    : DSocket(port, lfp) {}

    bool init()
    {
        // Opening standard UDP socket
        if ( (listen_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
            fprintf(lfp, "Error opening UDP socket.\n");
            return false;
        }
        // Set up listening address
        serv_addr.sin_family      = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY; 
        serv_addr.sin_port        = htons(port);
        // Bind
        int errNo = -1;
        if ( (errNo = bind(listen_fd, (const struct sockaddr*) &serv_addr, sizeof(serv_addr)) < 0)) {
            fprintf(lfp, "Error binding UDP socket. Error %d\n", errNo);
            return false;
        }
        fprintf(lfp, "UDP socket initialized successfully.\n");
        return true;
    }

    /*** IMPORTANT: the buffer returned must be malloc'd if needed, ***/
    /*** since this function zeroes it out each time ******************/
    char* msg_wait()
    {
        // Zero out buffer to ensure null-terminated
        memset(buffer, 0, MAXLINE);
        // Store incoming message in buffer
        socklen_t len = sizeof(client_addr);
        int m_size = recvfrom(listen_fd, (char*) buffer, MAXLINE, 0, 
                                (struct sockaddr*) &client_addr, &len);
        if (m_size > 0) {
            fprintf(lfp, " Got message: %s\n", buffer);
        } else {
            fprintf(lfp, "recv error %d\n", errno);
        }
        return buffer; 
    }
};



/*** UNIT-TEST ***/
#ifdef SOCKSERV

int main() {
    ServerSocket* serv_sock = new ServerSocket(ROSPORT);
    if (serv_sock->init()) {
        fprintf(stdout, "Waiting on port %d \n", serv_sock->port);
        fflush(stdout);
        char cmd;
        for (;;) {
            cmd = *serv_sock->msg_wait();
            fprintf(stdout, "Got command %c", cmd);
        }
    }
    return 0;
}

#endif // SOCKSERV

