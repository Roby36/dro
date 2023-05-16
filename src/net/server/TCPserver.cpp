
#include "TCPserver.h"

bool TCPserver::init()
{
    // Opening standard TCP socket
    if ( (listen_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
        fprintf(lfp, "Error opening TCP socket.\n");
        return false;
    }
    // Set up listening address
    serv_addr.sin_family      = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY; 
    serv_addr.sin_port        = htons(port);
    // Bind
    int errNo = -1;
    if ( (errNo = bind(listen_fd, (const struct sockaddr*) &serv_addr, sizeof(serv_addr)) < 0)) {
        fprintf(lfp, "Error binding TCP socket. Error %d\n", errNo);
        return false;
    }
    fprintf(lfp, "TCP socket binded successfully.\n");
    // Listen (TCP ONLY)
    if ( (errNo = listen(listen_fd, backlog) < 0)) {
        fprintf(lfp, "TCP socket listen error. Error %d\n", errNo);
        return false;
    }
    fprintf(lfp, "TCP socket listening successfully.\n");
    return true;
}

/*** IMPORTANT: the buffer returned must be malloc'd if needed, ***/
/*** since this function zeroes it out each time ******************/
char* TCPserver::msg_wait()
{
    // Zero out buffer to ensure null-terminated
    memset(buffer, 0, maxBuffSize);
    // Store incoming message in buffer
    socklen_t len = sizeof(client_addr);
    // Accept (TCP only)
    conn_fd = accept( listen_fd, (struct sockaddr*) &client_addr, &len);
    if ( conn_fd < 0) {
        fprintf(lfp, "TCP socket accept error. Error %d\n", errno);
        return buffer;
    }
    // Read data coming from client (TCP only)
    int m_size = read(conn_fd, (char*) buffer, maxBuffSize);
    if (m_size > 0) {
        // Log message received & time-stamp
        fprintf(lfp, " Got message '%s' from client at time %ld.\n\tDelay: %ld\n", 
            buffer,
            sys_time(),
            compute_time_diff(maxBuffSize, buffer, lfp));
    } else {
        fprintf(lfp, "read error %d\n", errno);
    }
    return buffer; 
}

void TCPserver::msg_respond_timestamped(char* msg)
{
    if (conn_fd < 0) {
        fprintf(lfp, " Cannot send message: client connection was unsuccessful\n");
        return;
    }
    // Append timestamp to message
    append_timestamp(msg);
    // Attempt to send message
    int m_size = write(conn_fd, msg, strlen(msg));
    if (m_size > 0) {
        // Log message sent
        fprintf(lfp, " Sent message '%s' to client.\n", msg);
    } else {
        fprintf(lfp, "write error %d\n", errno);
    }
}


