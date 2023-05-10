
#include "DSocket.h"

class ClientSocket : public DSocket
{
    const char* addr_string;

    public:

    ClientSocket(const char* addr_string, int port = SERVERPORT, FILE* lfp = stdout) 
    : DSocket(port, lfp), addr_string(addr_string) {}

    bool init()
    {
        // Opening standard UDP socket
        if ( (listen_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
            fprintf(lfp, "Error opening UDP socket.\n");
            return false;
        }
        // Set server address family and port
        serv_addr.sin_family      = AF_INET; 
        serv_addr.sin_port        = htons(port);
        // Enter server address using given string
        if (inet_pton(AF_INET, addr_string, &serv_addr.sin_addr) <= 0) {
            fprintf(lfp, "inet_pton error for %s\n", addr_string);
            return false;
        }
        fprintf(lfp, "UDP socket initialized successfully.\n");
        return true;
    }

    void send_timestamped(char* msg)
    {
        append_timestamp(msg);
        int m_size = sendto(listen_fd, msg, strlen(msg), 0, 
                            (const struct sockaddr*) &serv_addr, sizeof(serv_addr));
        if (m_size > 0) {
            fprintf(lfp, " Sent message: %s\n", msg);
        } else {
            fprintf(lfp, "recv error %d\n", errno);
        }
    }

    //*** Function adding timestamp at the end of message ****//
    static void append_timestamp(char* msg)
    {
        // Get current system time in seconds
        long now;
        sys_time(&now);
        // Insert the time at the end of the message preceded by a tab
        char str[MAXTIMEDIGITS];
        snprintf(str, MAXTIMEDIGITS, "\t%ld", now);
        strcat(msg, str);
    }

};


/*** UNIT-TEST ***/
#ifdef SOCKCLI

int main(int argc, char** argv) {
    // Usage check
    if (argc != 2) {
        printf("Usage: %s <server address>\n", argv[0]);
        exit(1);
    }
    ClientSocket* client_sock = new ClientSocket(argv[1], ROSPORT);
    // Initialize a buffer to be sent
    char buff[12];
    if (client_sock->init()) {
        for(;;) {
            // IMPORTANT: Truncate buffer each time to reset it
            buff[0] = '\0';
            snprintf(buff, 10, "Hello!\n");
            client_sock->send_timestamped(buff);
        }
    }
    return 0;
}

#endif // SOCKCLI
