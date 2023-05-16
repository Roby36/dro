
#include "UDPserver.h"

bool UDPserver::init()
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
char* UDPserver::msg_wait()
{
    // Zero out buffer to ensure null-terminated
    memset(buffer, 0, maxBuffSize);
    // Store incoming message in buffer
    socklen_t len = sizeof(client_addr);
    int m_size = recvfrom(listen_fd, (char*) buffer, maxBuffSize, 0, 
                            (struct sockaddr*) &client_addr, &len);
    if (m_size > 0) {
        fprintf(lfp, " Got message '%s' at time %ld.\n\tDelay: %ld\n", 
            buffer,
            sys_time(),
            compute_time_diff(maxBuffSize, buffer, lfp));
    } else {
        fprintf(lfp, "recv error %d\n", errno);
    }
    return buffer; 
}


