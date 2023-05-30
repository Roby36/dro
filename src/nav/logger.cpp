
#include "logger.h"

Logger::Logger(const std::string logpath)
{
    // Initialize logging filepath
    logfile = fopen(logpath.c_str(), "a");
    if (logfile == NULL) {
        ROS_INFO("Error: cannot open logging file %s\n",
                                    logpath.c_str());
    } else {
        ROS_INFO("Logging file %s opened successfully\n", 
                                    logpath.c_str());
    }
}

Logger::~Logger()
{
    // Close logging filepath
    if (logfile != NULL) {
        fclose(logfile);
        ROS_INFO("Logging file closed");
    }
}

bool Logger::logstr(const std::string str)
{
    // Log to rosout and/or logging file, based on compiler flags
    #ifdef NAVLOUT
        ROS_INFO("%s", str.c_str());
    #endif // NAVLOUT
    #ifdef NAVLFILE
        if (logfile == NULL) {
            return false; // meaning logfile couldn't be opened successfully at contruction
        }
        fprintf(logfile, "%s", str.c_str());
    #endif // NAVLFILE
        return true; // meaning we printed successfully to log file
}
