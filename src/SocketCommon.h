
#ifndef _SOCKETCOMMON_H_
#define _SOCKETCOMMON_H_

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
#include <unistd.h>

#define MAXLINE 16
#define SERVERPORT 7777

#endif  // _SOCKETCOMMON_H_