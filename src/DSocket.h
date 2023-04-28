
#ifndef _DSOCKET_H_
#define _DSOCKET_H_

#include "SocketCommon.h"

class DSocket
{
    protected:
    
    FILE* lfp;
    int listen_fd = 0;
    struct sockaddr_in serv_addr;

    public:

    DSocket(int port = SERVERPORT, FILE* lfp = stdout) 
    : port(port), lfp(lfp) {}

    const int port;
    virtual bool init() = 0;

    ~DSocket() { close(listen_fd); }
};

#endif // _SOCKET_H_