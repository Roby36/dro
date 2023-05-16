
CXX         = g++
CLIENTDIR   = ../src/net/client
COMMONDIR   = ../src/net/common
SERVERDIR   = ../src/net/server
NETLIB      = ../src/net/net.a
CXXFLAGS    = -std=c++11 -I$(CLIENTDIR) -I$(COMMONDIR) -I$(SERVERDIR)

JControllerClient: JControllerClient.cpp #UDPclient.h
	$(CXX) $(CXXFLAGS) $< $(NETLIB) -o $@ -lncurses

clean:
	rm -f *.o
	rm -f JControllerClient