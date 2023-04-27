
#include "ClientSocket.cpp"

int main(int argc, char** argv) {
    // Usage check
    if (argc != 2) {
        printf("Usage: %s <server address>\n", argv[0]);
        exit(1);
    }
    ClientSocket* client_sock = new ClientSocket(argv[1]);
    if (client_sock->init()) {
        for(;;) {
            client_sock->send("Hello!\n");

        }
    }
    return 0;
}
