
#include "PubHandler.h"

template <typename T> PubHandler<T>::PubHandler(ros::NodeHandle* nh, 
        const std::string topicName, const int queueSize)
    : topicName(topicName), queueSize(queueSize)
{
    // Initialize publisher
    this->pub = nh->advertise <T> (topicName, queueSize); 
}

template <typename T> void PubHandler<T>::publish(const T& msg)
{
    this->pub.publish(msg);
}

// Initialize explicitly all required classes for compiler
template class PubHandler <geometry_msgs::TwistStamped>;