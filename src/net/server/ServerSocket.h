
#pragma once
#include "SocketCommon.h"

class ServerSocket
{
    protected:

    //! Common to all sockets
    FILE* lfp;
    int listen_fd = 0;
    struct sockaddr_in serv_addr;

    //! ServerSocket attributes
    const int maxBuffSize;
    char* buffer;
    struct sockaddr_in client_addr;

    public:

    //! Common to all sockets
    const int port;

    //! General constructor & destructor
    ServerSocket( int port, int maxBuffSize, FILE* lfp)
        : port(port), lfp(lfp), maxBuffSize(maxBuffSize) 
    {
        this->buffer = new char[maxBuffSize];
    }
    
    virtual ~ServerSocket() {
        delete(this->buffer);
        close(listen_fd);
    }

    //! Virtual methods (depending on protocol used)
    virtual bool init()      = 0;
    virtual char* msg_wait() = 0;

};