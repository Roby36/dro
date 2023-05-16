
CXX       = g++
COMMONDIR = ../common
CXXFLAGS  = -std=c++11  -I$(COMMONDIR)
OBJS      = UDPserver.o TCPserver.o $(COMMONDIR)/SocketCommon.o

all: unittests

unittests: UDPserver TCPserver

TCPserver: serverTest.cpp UDPserver.h TCPserver.h $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -DTCPSERV $< -o $@

UDPserver: serverTest.cpp UDPserver.h TCPserver.h $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -DUDPSERV $< -o $@

TCPserver.o: TCPserver.cpp TCPserver.h ServerSocket.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

UDPserver.o: UDPserver.cpp UDPserver.h ServerSocket.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f *.o 
	rm -f UDPserver
	rm -f TCPserver