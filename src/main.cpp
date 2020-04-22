#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include "blueCougar_multiple_ros.h"

// trigger pin = digIn0+
// trigger ground level is same with the Arduino ground level.
// TODO: recognize serial numbers of multiple cameras and grant specific "id".
// TODO: multiple cameras setting is not supported yet .. T.T
int main(int argc, char **argv) {
    std::cout << "Bluecougar node is running..." << std::endl;
    ros::init(argc, argv, "bluecougar_multiple_node");
    ros::NodeHandle nh("~");

    bool binning_on   = true;
    bool triggered_on = false;
    bool aec_on       = true; // auto exposure control on / off
    bool agc_on       = true; // auto gain control on / off
    int expose_us     = 10000; // it is also max. exposure for auto exposure control.
    double frame_rate = 20.0; // frame rate (full resolution: up to 30 Hz)

    ros::param::get("~binning_on", binning_on);
	ros::param::get("~triggered_on", triggered_on);
	ros::param::get("~aec_on", aec_on);
	ros::param::get("~agc_on", agc_on);
	ros::param::get("~expose_us", expose_us);
    ros::param::get("~frame_rate", frame_rate);

    
    BlueCOUGAR_MULTIPLE_ROS *bluecougars = 
        new BlueCOUGAR_MULTIPLE_ROS(nh, binning_on, triggered_on, 
        aec_on, agc_on, expose_us, frame_rate);
    
    while(ros::ok()){
        bluecougars->Publish();
        ros::spinOnce();
    }

    delete bluecougars;
    ROS_INFO_STREAM("CEASE it.");
    return -1;
};