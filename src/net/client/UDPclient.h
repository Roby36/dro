
#pragma once
#include "ClientSocket.h"

class UDPclient : public ClientSocket
{
    public:

    UDPclient(const char* addr_string, int port = SERVERPORT, FILE* lfp = stdout)
        : ClientSocket(addr_string, port, lfp)
    {
    }

    ~UDPclient()
    {
    }

    virtual bool init();
    virtual void send_timestamped(char* msg);
};