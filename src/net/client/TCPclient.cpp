
#include "TCPclient.h"

bool TCPclient::init()
{
    // Opening standard TCP socket
    if ( (listen_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
        fprintf(lfp, "Error opening TCP socket.\n");
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
    fprintf(lfp, "TCP socket initialized successfully.\n");
    // Connect to server (TCP only)
    int res = connect(listen_fd,
                    (const struct sockaddr*) &serv_addr, 
                    sizeof(serv_addr));
    if (res < 0) {
        fprintf(lfp, " TCP socket connect error %d\n", errno);
        return false;
    }
    return true;
}

void TCPclient::send_timestamped(char* msg)
{
    // Append timestamp to message
    append_timestamp(msg);
    // Attempt to send message
    int m_size = write(listen_fd, msg, strlen(msg));
    if (m_size > 0) {
        // Log message sent
        fprintf(lfp, " Sent message '%s' to server.\n", msg);
    } else {
        fprintf(lfp, "write error %d\n", errno);
    }
}

char* TCPclient::msg_receive()
{
    // Read data coming from server (TCP only)
    int m_size = read(listen_fd, (char*)buffer, maxBuffSize);
    if (m_size > 0) {
        // Log message received & time-stamp
        fprintf(lfp, " Got message '%s' from server at time %ld.\n\tDelay: %ld\n", 
            buffer,
            sys_time(),
            compute_time_diff(maxBuffSize, buffer, lfp));
    } else {
        fprintf(lfp, "read error %d\n", errno);
    }
    return buffer; 
}
