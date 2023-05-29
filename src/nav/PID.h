
#pragma once
#include <cmath>
#include <algorithm>
#include <math.h>
#include <stdio.h>

#include "logger.h"

// Recording errors through an array is a limitation
// but provides lower-level simplicity for this first implementation of the module
#define MAXTIMESTEPS 1000000

// Define custom types for the input / output of PID
// The types needs to have the required operators overloaded / defined
typedef double unit_t;

class PID
{
    /** Logging **/
    Logger m_logger;

    //** PID terms **//
    unit_t K, Kp, Ki, Kd;

    //** Error-tracking **//
    // how many terms to consider in the error sum
    // defaults to -1, considering all the currently available error values
    int    err_sum_terms; 
    int    curr_err_term = 0;       // track current index of array
    unit_t err_array[MAXTIMESTEPS]; // array logging all previous errors

    //** Helper to get current error sum **//
    unit_t curr_err_sum();

    public:

    //** Constructor & destructor **//
    PID( const unit_t K, 
         const unit_t Kp, 
         const unit_t Ki, 
         const unit_t Kd, 
         const int err_sum_terms = -1, 
         const std::string logpath = "/root/catkin_ws/ardupilot_ws/src/dro/logs/PID_log.txt");
    
    ~PID();
    
    //** Function mapping error to command / action **//
    unit_t step(unit_t next_err, unit_t time_step);

    //** Function to assess PID performance **//
    void abs_err_sum(int& num_errors, unit_t& err_sum);

    //** Generate graph inputs **//
    bool log2file( std::string filename,
                   std::string filedir  = "/root/catkin_ws/ardupilot_ws/src/dro/graphs/input/");
                   
    //** Reset PID **//
    void reset(const unit_t K,  const unit_t Kp, 
               const unit_t Ki, const unit_t Kd, 
               const int err_sum_terms);
};