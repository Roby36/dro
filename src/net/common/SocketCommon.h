
#pragma once

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdarg.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <sys/types.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <ctime>
#include <chrono>

// Socket unit-tests
#define MAXTIMEDIGITS 24
#define TESTBUFFSIZE  64
#define SERVERPORT    7777

// ROS port-mapping
#define CMDVELPORT    7778
#define SERVICEPORT   7779

// Time-stamp functions
long sys_time();
void append_timestamp(char* msg);
void remove_timestamp(char* msg);
long compute_time_diff(const int maxBuffSize, 
                       char* buffer,     /*char (&buffer)[BUFFLIMIT]*/
                       FILE* lfp);
