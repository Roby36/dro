
#pragma once
#include "ServerSocket.h"

class TCPserver : public ServerSocket
{
    //! TCP server only private attributes
    const int backlog = 10;
    int conn_fd       = -1;

    public:

    TCPserver(int port = SERVERPORT, int maxBuffSize = 8 + MAXTIMEDIGITS,
        FILE* lfp = stdout)
        : ServerSocket(port, maxBuffSize, lfp)
    {
    }

    ~TCPserver()
    {
        close(conn_fd);
    }

    virtual bool init();
    virtual char* msg_wait();

    //! TCP server only method
    void msg_respond_timestamped(char* msg);
};