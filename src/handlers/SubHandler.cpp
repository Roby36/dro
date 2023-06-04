
#include "SubHandler.h"

template <typename T> SubHandler<T>::SubHandler (ros::NodeHandle* nh, 
    const std::string topicName, const int queueSize)
    : topicName(topicName), queueSize(queueSize)
{
    // Initialize subscriber
    this->sub = nh->subscribe( this->topicName, this->queueSize, &SubHandler::callback, this);
}

template <typename T> void SubHandler<T>::callback( const T& msg ) 
{
    // Assume memcpy returns successfully
    // Currently no check for initialization
    memcpy( &this->msg, &msg, sizeof(T));
    // Signal initialization upon recival of first message
    if (!initialized) {
        this->initialized = true;
    }
}

template <typename T> void SubHandler<T>::update_msg()
{
    do {
        ros::spinOnce();
    } while (!this->initialized);
}

// Initialize explicitly all required classes for compiler
template class SubHandler <nav_msgs::Odometry>;
template class SubHandler <sensor_msgs::LaserScan>;
template class SubHandler <sensor_msgs::Range>;
