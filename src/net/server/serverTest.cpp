
#include "UDPserver.h"
#include "TCPserver.h"

/*** UNIT-TEST ***/
int main() 
{
    #if defined(UDPSERV)
        UDPserver* serv_sock = new UDPserver(SERVERPORT, TESTBUFFSIZE);
    #elif defined(TCPSERV)
        TCPserver* serv_sock = new TCPserver(SERVERPORT, TESTBUFFSIZE);
    #else
        printf(" Socket type undefined.\n");
        exit(2);
    #endif

    if (serv_sock->init()) {
        fprintf(stdout, "Waiting on port %d \n", serv_sock->port);
        fflush(stdout);
        char buff[TESTBUFFSIZE]; 
        char cmd;
        for (;;) {
            strncpy(buff, serv_sock->msg_wait(), TESTBUFFSIZE);

            // TCP servers confirm message reception
            #ifdef TCPSERV
            // First remove the time-stamp from the message
            remove_timestamp(buff);
            char msg[TESTBUFFSIZE + MAXTIMEDIGITS];
            snprintf(msg, 
                TESTBUFFSIZE + MAXTIMEDIGITS, 
                "Server received message '%s' (without timestamp)\n", buff);
            serv_sock->msg_respond_timestamped(msg);
            // For TCP break loop after responding
            break;
            #endif
        }
    }
    delete(serv_sock);
    return 0;
}

