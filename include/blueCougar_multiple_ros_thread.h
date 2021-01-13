#ifndef _BLUECOUGAR_MULTIPLE_ROS_HHI_H_
#define _BLUECOUGAR_MULTIPLE_ROS_HHI_H_

#include <iostream>
#include <vector>
#include <sys/time.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
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


#include <thread>
#include <mutex>
#include <memory> 

#include "bluecougar.h"

using namespace std;
using namespace mvIMPACT::acquire;
using namespace mvIMPACT::acquire::GenICam;


class BlueCOUGAR_MULTIPLE_ROS_THREAD_HHI {
public:
    explicit BlueCOUGAR_MULTIPLE_ROS_THREAD_HHI(
        ros::NodeHandle& nh, bool binning_on, bool triggered_on,
        bool aec_on, bool agc_on, int expose_us, double frame_rate)
    : nh_(nh), it_(nh_)
    {
        n_devs_ = getValidDevices(devMgr_, validDevices_);
        std::cout << "[BlueCOUGAR multiple info] # of valid devices: " << n_devs_ << std::endl;
        // show devices information
        for(int i = 0; i < n_devs_; i++){
            std::string topic_name = "/" + std::to_string(i) + "/image_raw";
            std::cout << "[" << i << "]: ";

            BlueCougar* bc_tmp = 
                new BlueCougar(validDevices_[i], i, binning_on, triggered_on, 
                        aec_on, agc_on, expose_us, frame_rate);

            bluecougars_.push_back(bc_tmp);

            // publishers
            image_transport::Publisher camera_pub_ = it_.advertise(topic_name,3);
            image_publishers_.push_back(camera_pub_);
            img_msgs_.push_back(sensor_msgs::Image());
        }

        // start live threads
        for( auto* bc : bluecougars_){
            bc->startThread( liveThread, bc );
        }

        sub_msg_ = nh_.subscribe("/hhi/msg", 1, &BlueCOUGAR_MULTIPLE_ROS_THREAD_HHI::callbackHHI, this);
        cout << "[BlueCOUGAR multiple info] Please wait for setting cameras...\n";
        const unsigned int sleepPeriod_ms = 1000;
        this_thread::sleep_for( chrono::milliseconds( sleepPeriod_ms ) );
        cout << "[BlueCOUGAR multiple info] camera setting is done.\n";
    };    
    ~BlueCOUGAR_MULTIPLE_ROS_THREAD_HHI();

    void callbackHHI(const std_msgs::Int32::ConstPtr& msg);
    int getStatus() const { return msg_.data;};

private:
    int n_devs_; // # of connected mvBlueCOUGAR cameras.
    mvIMPACT::acquire::DeviceManager devMgr_; // Manager for all devices.

    vector<mvIMPACT::acquire::Device*> validDevices_; // multiple devices   
    vector<BlueCougar*> bluecougars_; // bluecougars 
    
    // For ros.
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    vector<image_transport::Publisher> image_publishers_;
    vector<sensor_msgs::Image> img_msgs_;

    ros::Subscriber sub_msg_;
    std_msgs::Int32 msg_;
};

/* IMPLEMENTATION */
BlueCOUGAR_MULTIPLE_ROS_THREAD_HHI::~BlueCOUGAR_MULTIPLE_ROS_THREAD_HHI() {
    // join all threads


    // delete bluecougar objects
    for(int i = 0; i < n_devs_; i++){
        bluecougars_[i]->terminateThread();
        delete bluecougars_[i];
    }
};

void BlueCOUGAR_MULTIPLE_ROS_THREAD_HHI::callbackHHI(const std_msgs::Int32::ConstPtr& msg)
{
    msg_.data = msg->data;
    bool state_grab = true;
    
    const unsigned int sleepPeriod_ms = 1000;
    this_thread::sleep_for( chrono::milliseconds( sleepPeriod_ms ) );
    
    if(msg_.data == 1){ // snapshot grab mode.
        for(int i = 0; i < n_devs_; i++){
            state_grab = state_grab & bluecougars_[i]->grabImageThread(img_msgs_[i]);
        }   
        if(state_grab){
            for(int i = 0; i <n_devs_; i++){
                //image_publishers_[i].publish(img_msgs_[i]);
            }
        }
    }
    else if(msg_.data == 2){ // set exposure time [us]
        for(int i = 0; i < n_devs_; i++){
            bluecougars_[i]->setExposureTime(3000);
        }
    }
    else if(msg_.data == 3){ // set gain
    }
};
#endif
