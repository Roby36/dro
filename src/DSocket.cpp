
#include "DSocket.h"

DSocket::DSocket(int port, FILE* lfp)
        : port(port), lfp(lfp) 
{
}

DSocket::~DSocket() { 
    close(listen_fd);
}

void DSocket::sys_time(long* ts) {
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    *ts = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
}