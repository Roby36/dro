
#pragma once
#include "SocketCommon.h"

class ClientSocket
{
    protected:

    //! Common to all sockets
    FILE* lfp;
    int listen_fd = 0;
    struct sockaddr_in serv_addr;

    //! ClientSocket attributes
    const char* addr_string;

    public:

    //! Common to all sockets
    const int port;

    //! Constructor & destructor
    ClientSocket(const char* addr_string, int port, FILE* lfp)
        : port(port), lfp(lfp), addr_string(addr_string)
    {
    }

    virtual ~ClientSocket() {
        close(listen_fd);
    }

    //! Virtual methods (depending on protocol used)
    virtual bool init()                      = 0;
    virtual void send_timestamped(char* msg) = 0;
    
};