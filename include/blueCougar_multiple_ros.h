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


using namespace std;
using namespace mvIMPACT::acquire;
using namespace mvIMPACT::acquire::GenICam;

class BlueCOUGAR_MULTIPLE_ROS {
public:
    explicit BlueCOUGAR_MULTIPLE_ROS(
        ros::NodeHandle& nh, bool binning_on, bool triggered_on,
        bool aec_on, bool agc_on, int expose_us, double frame_rate)
    : nh_(nh), it_(nh_)
    {
        n_devs_ = getValidDevices(devMgr_, validDevices_);
        std::cout << "# of valid devices: " << n_devs_ << std::endl;
        // show devices information
        for(int i = 0; i < n_devs_; i++){
            std::cout << "[" << i << "]: ";
            BlueCougar* bluecougar_temp = 
            new BlueCougar(validDevices_[i], i, binning_on, triggered_on, 
                        aec_on, agc_on, expose_us, frame_rate);
            std::string topic_name = "/" + std::to_string(i) + "/image_raw";

            bluecougars_.push_back(bluecougar_temp);

            image_transport::Publisher camera_pub_ = it_.advertise(topic_name,1);
            image_publishers_.push_back(camera_pub_);
            img_msgs_.push_back(sensor_msgs::Image());
        }
        std::cout<< "Please wait for setting cameras...\n"<<std::endl;
        sleep(2);
    };    
    ~BlueCOUGAR_MULTIPLE_ROS();

    std::vector<sensor_msgs::Image> img_msgs_;
    void Publish();

private:
    int n_devs_; // # of connected mvBlueCOUGAR cameras.
    ros::NodeHandle nh_; // node handler for ROS publish.
    mvIMPACT::acquire::DeviceManager devMgr_; // Manager for all devices.
    vector<mvIMPACT::acquire::Device*> validDevices_; // multiple devices
    vector<BlueCougar*> bluecougars_;

    vector<image_transport::Publisher> image_publishers_;
    vector<image_transport::ImageTransport> img_transports_;

    image_transport::ImageTransport it_;
    image_transport::Publisher camera_pub_;  
};

/* IMPLEMENTATION */
BlueCOUGAR_MULTIPLE_ROS::~BlueCOUGAR_MULTIPLE_ROS(){
    for(int i = 0; i < n_devs_; i++){
        delete bluecougars_[i];
    }
};
//const sensor_msgs::ImagePtr& image_msg
void BlueCOUGAR_MULTIPLE_ROS::Publish() {
    for(int i = 0; i < n_devs_; i++){
        bluecougars_[i]->grabImage(img_msgs_[i]);
    }   
    for(int i = 0; i <n_devs_; i++){
        image_publishers_[i].publish(img_msgs_[i]);
    }
};

#endif
