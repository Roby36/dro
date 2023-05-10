
#pragma once
#include "SocketCommon.h"

class DSocket
{
    protected:
    
    FILE* lfp;
    int listen_fd = 0;
    struct sockaddr_in serv_addr;

    //*** For adding time-stamps ***//
    static void sys_time(long*);

    public:

    const int port;
    virtual bool init() = 0;

    DSocket(int port = SERVERPORT, FILE* lfp = stdout);
    ~DSocket();

};
