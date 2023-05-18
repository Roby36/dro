
#include "ServiceHandler.h"

ServiceHandler::ServiceHandler( ros::NodeHandle* nh) {
    /* Initialize service clients */
    this->arming_srv  = nh->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    this->takeoff_srv = nh->serviceClient<mavros_msgs::CommandTOL> ("/mavros/cmd/takeoff");
    this->land_srv    = nh->serviceClient<mavros_msgs::CommandTOL> ("/mavros/cmd/land");
    this->mode_srv    = nh->serviceClient<mavros_msgs::SetMode>    ("/mavros/set_mode");
}

bool 
ServiceHandler::reqMode(BaseMode base_mode, std::string custom_mode)
{
    mavros_msgs::SetMode::Request  req;
    mavros_msgs::SetMode::Response resp;
    req.base_mode   = base_mode;
    req.custom_mode = custom_mode;

    bool success = this->mode_srv.call(req, resp);
    if (!resp.mode_sent) {
        ROS_ERROR_STREAM("Failed to send base mode "  << base_mode << "; "  << "custom mode " << custom_mode);
    } else if (!success) {
        ROS_ERROR_STREAM("Failed to set base mode "   << base_mode << "; "  << "custom mode " << custom_mode);
    } else {
        ROS_INFO_STREAM("Successfully set base mode " << base_mode << "; "  << "custom mode " << custom_mode);
    }
}

bool 
ServiceHandler::reqArming(bool value) 
{
    mavros_msgs::CommandBool::Request  req;
    mavros_msgs::CommandBool::Response resp;
    req.value = value;

    bool success = this->arming_srv.call(req, resp);
    if (resp.success) {
        ROS_INFO_STREAM("Arming status successfully changed: result: " << resp.result);
    } else {
        ROS_ERROR_STREAM("Failed to change arming status: result: "    << resp.result);
    }
    return resp.success;
}

bool 
ServiceHandler::reqTakeoff(float min_pitch, float yaw, float latitude, 
                           float longitude, float altitude)
{
    mavros_msgs::CommandTOL::Request  req;
    mavros_msgs::CommandTOL::Response resp;
    req.min_pitch = min_pitch;
    req.yaw       = yaw;
    req.latitude  = latitude;
    req.longitude = longitude;
    req.altitude  = altitude;

    bool success = this->takeoff_srv.call(req, resp);
    if (resp.success) {
        ROS_INFO_STREAM("Takeoff successful: result: " << resp.result);
    } else {
        ROS_ERROR_STREAM("Failed to takeoff: result: " << resp.result);
    }
    return resp.success;
}

bool 
ServiceHandler::reqLand(float min_pitch, float yaw, float latitude, 
                        float longitude, float altitude)
{
    mavros_msgs::CommandTOL::Request  req;
    mavros_msgs::CommandTOL::Response resp;
    req.min_pitch = min_pitch;
    req.yaw       = yaw;
    req.latitude  = latitude;
    req.longitude = longitude;
    req.altitude  = altitude;
    
    bool success = this->land_srv.call(req, resp);
    if (resp.success) {
        ROS_INFO_STREAM("Landing successful: result: " << resp.result);
    } else {
        ROS_ERROR_STREAM("Failed to land: result: "    << resp.result);
    }
    return resp.success;
}

