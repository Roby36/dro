
#include "UDPclient.h"
#include "TCPclient.h"

/*** UNIT-TEST ***/
int main(int argc, char** argv) 
{
        // const int numMessages = 100000;
    // Usage check
    if (argc != 2) {
        printf("Usage: %s <server address>\n", argv[0]);
        exit(1);
    }

    #if defined(UDPCLI)
    UDPclient* client_sock = new UDPclient(argv[1], SERVERPORT);
    #elif defined(TCPCLI)
    // client_sock = dynamic_cast <TCPclient*> (new TCPclient(argv[1], SERVERPORT));
    TCPclient* client_sock = new TCPclient(argv[1], 
        TESTBUFFSIZE + MAXTIMEDIGITS, SERVERPORT);
    #else
    printf(" Socket type undefined.\n");
    exit(2);
    #endif 
    
    // Initialize a buffer to be sent
    char buff[TESTBUFFSIZE];
    if (client_sock->init()) {
        for(;;) {
            // Control loop speed
                // usleep(1000);
            // IMPORTANT: Truncate buffer each time to reset it
            buff[0] = '\0';
            snprintf(buff, TESTBUFFSIZE, "Hello!");
            client_sock->send_timestamped(buff);

            #if defined(TCPCLI)
            printf(" Waiting for server's response\n");
            client_sock->msg_receive();
            // For TCP break as soon as we have received the response
            break;
            #endif 
        }
    }
    // Clean up
    delete(client_sock);
    return 0;
}

