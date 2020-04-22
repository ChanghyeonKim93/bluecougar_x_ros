#ifndef _BLUECOUGAR_MULTIPLE_ROS_H_
#define _BLUECOUGAR_MULTIPLE_ROS_H_

#include <iostream>
#include <vector>
#include <sys/time.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>

#include "bluecougar.h"

using namespace mvIMPACT::acquire;
using namespace mvIMPACT::acquire::GenICam;

class BlueCOUGAR_MULTIPLE_ROS{
    public:
    explicit BlueCOUGAR_MULTIPLE_ROS(ros::NodeHandle& nh, bool binning_on, 
    bool triggered_on, bool aec_on, bool agc_on, int expose_us, double frame_rate)
    : nh_(nh), it_(nh_), camera_pub_(it_.advertise("image_raw", 1))
    {
        num_devs_ = getValidDevices(devMgr_, validDevices_);
        std::cout <<"# of valid devices: "<< num_devs_ <<std::endl;
        // show devices information
        for(int i = 0; i <num_devs_; i++){
            std::cout<<"["<<i<<"]: ";
            BlueCougar* bluecougar_temp = new BlueCougar(validDevices_[i],binning_on, 
            triggered_on, aec_on, agc_on, expose_us, frame_rate);
            bluecougars_.push_back(bluecougar_temp);
        }
    };    
    ~BlueCOUGAR_MULTIPLE_ROS();

    sensor_msgs::Image msg_img_;
    void Publish();

    private:
    int num_devs_;
    ros::NodeHandle nh_;
    mvIMPACT::acquire::DeviceManager devMgr_;
    std::vector<mvIMPACT::acquire::Device*> validDevices_; // multiple devices
    std::vector<BlueCougar*> bluecougars_;

    image_transport::ImageTransport it_;
    image_transport::Publisher camera_pub_;  
};

/* IMPLEMENTATION */


BlueCOUGAR_MULTIPLE_ROS::~BlueCOUGAR_MULTIPLE_ROS(){
    for(int i = 0; i < num_devs_; i++){
        delete bluecougars_[i];
    }
};
//const sensor_msgs::ImagePtr& image_msg
void BlueCOUGAR_MULTIPLE_ROS::Publish() {
    bluecougars_[0]->grabImage(msg_img_);
    camera_pub_.publish(msg_img_);
};

#endif