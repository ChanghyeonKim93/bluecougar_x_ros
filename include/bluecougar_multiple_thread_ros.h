#ifndef _BLUECOUGAR_MULTIPLE_THREAD_ROS_H_
#define _BLUECOUGAR_MULTIPLE_THREAD_ROS_H_

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

#include <thread>
#include <mutex>
#include <condition_variable>

#include "bluecougar.h"

using namespace std;
using namespace mvIMPACT::acquire;
using namespace mvIMPACT::acquire::GenICam;

class BlueCOUGAR_MULTITHREAD_ROS {
public:
    explicit BlueCOUGAR_MULTITHREAD_ROS(
        ros::NodeHandle& nh, bool binning_on, bool triggered_on,
        bool aec_on, bool agc_on, int expose_us, double frame_rate) 
        : nh_(nh), it_(nh_)
    {
        n_devs_ = getValidDevices(devMgr_, validDevices_);
        cout << "# of valid devices: " << n_devs_ <<"\n";

        // show devices information
        for(int i = 0; i < n_devs_; i++) {
            cout << "[" << i << "]-th device:";

            BlueCougar* bluecougar_temp = 
            new BlueCougar(validDevices_[i], i, binning_on, triggered_on, 
                        aec_on, agc_on, expose_us, frame_rate);
            bluecougars_.push_back(bluecougar_temp);

            string topic_name = "/" + std::to_string(i) + "/image_raw";
            image_transport::Publisher camera_pub_ = it_.advertise(topic_name, 1);
            image_publishers_.push_back(camera_pub_);
            img_msgs_.push_back(sensor_msgs::Image());
        }

        // start threads

        cout << "Please wait for initialzing all cameras...\n\n\n";
        ros::Duration(3.0).sleep(); // 3 seconds
    };    
    ~BlueCOUGAR_MULTITHREAD_ROS() {
        for(int i = 0; i < n_devs_; i++) delete bluecougars_[i];
    };

    void runMultipleCameras();
    std::vector<sensor_msgs::Image> img_msgs_;

private:
    int n_devs_; // # of connected mvBlueCOUGAR cameras.
    mvIMPACT::acquire::DeviceManager devMgr_; // Manager for all devices.
    vector<mvIMPACT::acquire::Device*> validDevices_; // multiple devices
    vector<BlueCougar*> bluecougars_; // multiple devices

    // for ros
    ros::NodeHandle nh_; // node handler for ROS publish.
    vector<image_transport::Publisher> image_publishers_;
    vector<image_transport::ImageTransport> img_transports_;

    image_transport::ImageTransport it_;
    image_transport::Publisher camera_pub_;

    // subscriber for hhi_msg
    ros::Subscriber sub_cmd_msg_;

    // for multi thread
    condition_variable cv_;
    vector<mutex> mutexes_;
    vector<thread> cam_threads;

};

/* IMPLEMENTATION */
void BlueCOUGAR_MULTITHREAD_ROS::runMultipleCameras() {
    for(int i = 0; i < n_devs_; i++){
        bluecougars_[i]->grabImage(img_msgs_[i]);
    }   
    for(int i = 0; i <n_devs_; i++){
        image_publishers_[i].publish(img_msgs_[i]);
    }

    while(true){
        
    };
};

#endif