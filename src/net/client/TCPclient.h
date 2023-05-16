
#pragma once
#include "ClientSocket.h"

class TCPclient : public ClientSocket
{
    //! TCPclient only attrubute (buffer to store server response)
    const int maxBuffSize;
    char* buffer;

    public:

    TCPclient(const char* addr_string, int maxBuffSize = 16 + MAXTIMEDIGITS, int port = SERVERPORT, FILE* lfp = stdout)
        : ClientSocket(addr_string, port, lfp), maxBuffSize(maxBuffSize)
    {
        this->buffer = new char[maxBuffSize];
    }

    ~TCPclient()
    {
        delete(this->buffer);
    }

    virtual bool init();
    virtual void send_timestamped(char* msg);

    //! TCP only method
    char* msg_receive();
};