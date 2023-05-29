
/* Very simple logging class to handle logging
    for all classes in this module */

#pragma once
#include <ros/ros.h>
#include <stdio.h>

class Logger 
{
    FILE* logfile;

    public:

    Logger(const std::string logpath);
    ~Logger();
    bool logstr(const std::string str);

};