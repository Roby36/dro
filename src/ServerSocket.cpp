
#include "DSocket.h"

class ServerSocket : public DSocket
{
    const int maxBuffSize;
    char* buffer;
    struct sockaddr_in client_addr;

    public:

    ServerSocket(int port = SERVERPORT, int maxBuffSize = 8 + MAXTIMEDIGITS,
        FILE* lfp = stdout) : DSocket(port, lfp), maxBuffSize(maxBuffSize)
    {
        this->buffer = new char[maxBuffSize];
    }

    ~ServerSocket() {
        delete(this->buffer);
    }

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

    //*** Function extracting time from time-stamped buffer ****//
    //*** and computing the time difference using current system time ***//
    long compute_time_diff() {
        // Attempt to reach the tab at the end of the message
        char* p = buffer;
        int c = 0;
        while (*(++p) != '\t') {
            if (*p == '\0' || c ==  maxBuffSize - 2) {
                fprintf(lfp, " Error: could not locate time-stamp in buffer\n");
                return (long) -1;
            }
            c++;
        }
        // Copy the time string
        char str[MAXTIMEDIGITS];
        int i = 0;
        while (*(p++) != '\0') {
            str[i++] = *p;
            if (i == MAXTIMEDIGITS) {
                fprintf(lfp, " Error: reached max digits when copying time-stamp\n");
                return (long) -1;
            }
        }
        // Try to extract stamped time from string
        long tstamp;
        if (sscanf(str, "%ld", &tstamp) != 1) {
            fprintf(lfp, " Error: could not extract time-stamp from buffer\n");
            return (long) -1;
        }
        // Compute the time difference with the current time
        long now;
        sys_time(&now);
        return now - tstamp;
    }

    /*** IMPORTANT: the buffer returned must be malloc'd if needed, ***/
    /*** since this function zeroes it out each time ******************/
    char* msg_wait()
    {
        // Zero out buffer to ensure null-terminated
        memset(buffer, 0, maxBuffSize);
        // Store incoming message in buffer
        socklen_t len = sizeof(client_addr);
        int m_size = recvfrom(listen_fd, (char*) buffer, maxBuffSize, 0, 
                                (struct sockaddr*) &client_addr, &len);
        if (m_size > 0) {
            fprintf(lfp, " Got message: %s Delay: %ld\n", buffer, compute_time_diff());
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
            serv_sock->msg_wait();
            /*
            cmd = *serv_sock->msg_wait();
            fprintf(stdout, "Got command %c\n", cmd);
            */
        }
    }
    return 0;
}

#endif // SOCKSERV

