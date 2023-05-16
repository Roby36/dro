
CXX       = g++
COMMONDIR = ../common
CXXFLAGS  = -std=c++11  -I$(COMMONDIR)
OBJS      = UDPclient.o TCPclient.o  $(COMMONDIR)/SocketCommon.o

all: unittests

unittests: UDPclient TCPclient

TCPclient: clientTest.cpp UDPclient.h TCPclient.h $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -DTCPCLI $< -o $@

UDPclient: clientTest.cpp UDPclient.h TCPclient.h $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -DUDPCLI $< -o $@

UDPclient.o: UDPclient.cpp UDPclient.h ClientSocket.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

TCPclient.o: TCPclient.cpp TCPclient.h ClientSocket.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f *.o 
	rm -f UDPclient
	rm -f TCPclient