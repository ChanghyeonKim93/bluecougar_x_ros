#include <iostream>
#include <ros/ros.h>

#include <string>
#include <vector>

#include "blueCougar_multiple_ros_hhi.h"

using namespace std;

// trigger pin = digIn0+
// trigger ground level is same with the Arduino ground level.
// TODO: recognize serial numbers of multiple cameras and grant specific "id".
// TODO: multiple cameras setting is not supported yet .. T.T
int main(int argc, char **argv) {
    std::cout << "Bluecougar node is running..." << std::endl;
    ros::init(argc, argv, "bluecougar_multiple_node");
    ros::NodeHandle nh("~");

    bool binning_on   = true;
    bool triggered_on = true;

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

    vector<string> serials;
    serials.resize(4);
    ros::param::get("~serial_cam_boom0",  serials[0]);
    ros::param::get("~serial_cam_boom1",  serials[1]);
    ros::param::get("~serial_cam_cabin0", serials[2]);
    ros::param::get("~serial_cam_cabin1", serials[3]);
    
    //serials[0] = "GX033765"; //  boom0 (X104iG, gray )
    //serials[1] = "GX033832"; //  boom1 (X104iG, gray )
    //serials[2] = "GX032919"; // cabin0 (X104iC, color)
    //serials[3] = "GX032926"; // cabin1 (X104iC, color)
        
    BlueCOUGAR_MULTIPLE_ROS_HHI *bluecougars = 
       // new BlueCOUGAR_MULTIPLE_ROS_HHI(nh, binning_on, triggered_on, 
       // aec_on, agc_on, expose_us, frame_rate);
        new BlueCOUGAR_MULTIPLE_ROS_HHI(nh, binning_on, triggered_on, 
        aec_on, agc_on, expose_us, frame_rate, serials);


    while((bluecougars->getStatus() > -1) && ros::ok()) {
        ros::spinOnce();
    }

    delete bluecougars;
    ROS_INFO_STREAM("Cease cameras.\n");
    return -1;
};
