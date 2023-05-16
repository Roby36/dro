
CXX       = g++
CXXFLAGS  = -std=c++11
OBJS      = SocketCommon.o

all: $(OBJS)

SocketCommon.o: SocketCommon.cpp SocketCommon.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f *.o 