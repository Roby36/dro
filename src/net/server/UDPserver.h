
#pragma once
#include "ServerSocket.h"

class UDPserver : public ServerSocket
{
    public:

    UDPserver(int port = SERVERPORT, int maxBuffSize = 8 + MAXTIMEDIGITS,
        FILE* lfp = stdout)
        : ServerSocket(port, maxBuffSize, lfp)
    {
    }

    ~UDPserver()
    {
    }

    virtual bool init();
    virtual char* msg_wait();
};