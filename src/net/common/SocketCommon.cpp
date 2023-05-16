
#include "SocketCommon.h"

long sys_time() {
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    return (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
}

//*** Function adding timestamp at the end of message ****//
void append_timestamp(char* msg)
{
    // Get current system time in seconds
    long now = sys_time();
    // Insert the time at the end of the message preceded by a tab
    char str[MAXTIMEDIGITS];
    snprintf(str, MAXTIMEDIGITS, "\t%ld", now);
    strcat(msg, str);
}

//*** Function removing any timestamp at the end of message ****//
//*** IMPORTANT: If the messaage is malloc'd, this results in a memory leak ****//
void remove_timestamp(char* msg)
{
    char* p = msg;
    while (*(++p) != '\t') {
        if (*p == '\0') {
            printf(" remove_timestamp error: could not locate time-stamp in message\n");
            return;
        }
    }
    // Truncate the message at the first tab that is encountered
    *p = '\0';
}

//*** Function extracting time from time-stamped buffer ****//
//*** and computing the time difference using current system time ***//
long compute_time_diff(const int maxBuffSize, 
                       char* buffer,     /*char (&buffer)[BUFFLIMIT]*/
                       FILE* lfp) {
    // Attempt to reach the tab at the end of the message
    char* p = buffer;
    int c = 0;
    while (*(++p) != '\t') {
        if (*p == '\0' || c ==  maxBuffSize - 2) {
            fprintf(lfp, " Error: could not locate time-stamp in buffer\n");
            return (long) -1;
        }
        c++;
    }
    // Copy the time string
    char str[MAXTIMEDIGITS];
    int i = 0;
    while (*(p++) != '\0' ) {
        str[i++] = *p;
        if (i == MAXTIMEDIGITS) {
            fprintf(lfp, " Error: reached max digits when copying time-stamp\n");
            return (long) -1;
        }
    }
    // Try to extract stamped time from string
    long tstamp;
    if (sscanf(str, "%ld", &tstamp) != 1) {
        fprintf(lfp, " Error: could not extract time-stamp from buffer\n");
        return (long) -1;
    }
    // Compute the time difference with the current time
    long now = sys_time();
    return now - tstamp;
}

