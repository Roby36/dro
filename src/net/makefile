
SERVERDIR = ./server
CLIENTDIR = ./client
COMMONDIR = ./common
LIB 	  = net.a

# List all object files necessary for library here
OBJS = $(COMMONDIR)/SocketCommon.o \
       $(SERVERDIR)/UDPserver.o $(SERVERDIR)/TCPserver.o \
       $(CLIENTDIR)/UDPclient.o $(CLIENTDIR)/TCPclient.o

all: $(LIB)

$(LIB): # $(OBJS) # make each required object file
	cd $(COMMONDIR) && make SocketCommon.o
	cd $(SERVERDIR) && make UDPserver.o && make TCPserver.o
	cd $(CLIENTDIR) && make UDPclient.o && make TCPclient.o
	ar -cr $@ $(OBJS)  # archive all object files into one library

# Make each submodule'S unit-tests
unittests:
	make -C common SocketCommon.o  #first the common directory 
	make -C client $@
	make -C server $@ 
	
# Clean each submodule
clean:
	make -C common clean
	make -C server clean
	make -C client clean
