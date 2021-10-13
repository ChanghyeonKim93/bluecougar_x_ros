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

#include "bluecougar.h"

#include "dynamic_reconfigure/server.h"
#include "bluecougar/bluecougarDynConfig.h"

// GX033765 (1.13) boom0  (     )
// GX033832 (1.14) boom1  (     )
// GX032919 (2.30) cabin0 (lower)
// GX032926 (2.31) cabin1 (upper)

using namespace std;
using namespace mvIMPACT::acquire;
using namespace mvIMPACT::acquire::GenICam;

class BlueCOUGAR_MULTIPLE_ROS_HHI {
public:
    explicit BlueCOUGAR_MULTIPLE_ROS_HHI(
        ros::NodeHandle& nh, bool binning_on, bool triggered_on,
        bool aec_on, bool agc_on, int expose_us, double frame_rate)
    : nh_(nh), it_(nh_)
    {
        n_devs_ = getValidDevices(devMgr_, validDevices_);
        std::cout << "[BlueCOUGAR multiple info] # of valid devices: " << n_devs_ << std::endl;
        // show devices information
        for(int i = 0; i < n_devs_; i++) {
            std::cout << "[" << i << "]: ";
            BlueCougar* bluecougar_temp =
            new BlueCougar(validDevices_[i], i, binning_on, triggered_on, 
                        aec_on, agc_on, expose_us, frame_rate);
            std::string topic_name = "/" + std::to_string(i) + "/image_raw";

            bluecougars_.push_back(bluecougar_temp);

            image_transport::Publisher camera_pub_ = it_.advertise(topic_name,3);
            image_publishers_.push_back(camera_pub_);
            img_msgs_.push_back(sensor_msgs::Image());
        }
        sub_msg_ = nh_.subscribe("/hhi/msg", 1, &BlueCOUGAR_MULTIPLE_ROS_HHI::callbackHHI, this);
        cout << "[BlueCOUGAR multiple info] Please wait for setting cameras...\n";
        ros::Duration(1.0).sleep();
        cout << "[BlueCOUGAR multiple info] camera setting is done.\n";
    };    

    explicit BlueCOUGAR_MULTIPLE_ROS_HHI(
        ros::NodeHandle& nh, bool binning_on, bool triggered_on,
        bool aec_on, bool agc_on, int expose_us, double frame_rate,
        vector<string> serials_)
    : nh_(nh), it_(nh_)
    {
        n_devs_ = getValidDevices(devMgr_, validDevices_);
        std::cout << "[BlueCOUGAR multiple info] # of valid devices: " << n_devs_ << std::endl;
        // show devices information'
        for(int i = 0; i < n_devs_; ++i){
            string cur_serial = serials_[i];            
            int idx_temp = -1;
            for(int j = 0; j < n_devs_; j++) {
                cout << validDevices_[j]->serial.read() <<"\n";
                if(validDevices_[j]->serial.read() == cur_serial){
                    cout << "  idx    : " << j << endl;
                    cout << "  valdev :" << validDevices_[j]->serial.read() <<"\n";
                    cout << "  serial :" << cur_serial << "\n";
                    idx_temp = j;
                }
            }
            std::cout << "[" << i << "]: ";
            BlueCougar* bluecougar_temp =
                new BlueCougar(validDevices_[idx_temp], i, binning_on, triggered_on, 
                        aec_on, agc_on, expose_us, frame_rate);
            std::string topic_name = "/" + std::to_string(i) + "/image_raw";

            bluecougars_.push_back(bluecougar_temp);

            image_transport::Publisher camera_pub_ = it_.advertise(topic_name,3);
            image_publishers_.push_back(camera_pub_);
            img_msgs_.push_back(sensor_msgs::Image());
        }
        

        // dynamic reconfigure for real-time hardware parameter settings
        f_ = boost::bind(&BlueCOUGAR_MULTIPLE_ROS_HHI::callbackDynReconfig, this, _1, _2);
        server_.setCallback(f_);

        sub_msg_ = nh_.subscribe("/hhi/msg", 1, &BlueCOUGAR_MULTIPLE_ROS_HHI::callbackHHI, this);
        cout << "[BlueCOUGAR multiple info] Please wait for setting cameras...\n";
        ros::Duration(1.0).sleep();
        cout << "[BlueCOUGAR multiple info] camera setting is done.\n";
    };    

    ~BlueCOUGAR_MULTIPLE_ROS_HHI();

    void callbackDynReconfig(bluecougar::bluecougarDynConfig &config, uint32_t lvl);

    void callbackHHI(const std_msgs::Int32::ConstPtr& msg);
    int getStatus() const { return msg_.data; };

private:
    int n_devs_; // # of connected mvBlueCOUGAR cameras.
    mvIMPACT::acquire::DeviceManager devMgr_; // Manager for all devices.

    vector<mvIMPACT::acquire::Device*> validDevices_; // multiple devices
    vector<BlueCougar*> bluecougars_;
    
    // For ros.
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    vector<image_transport::Publisher> image_publishers_;
    vector<sensor_msgs::Image> img_msgs_;

    ros::Subscriber sub_msg_;
    std_msgs::Int32 msg_;

    dynamic_reconfigure::Server<bluecougar::bluecougarDynConfig> server_;
    dynamic_reconfigure::Server<bluecougar::bluecougarDynConfig>::CallbackType f_;
};

/* IMPLEMENTATION */
BlueCOUGAR_MULTIPLE_ROS_HHI::~BlueCOUGAR_MULTIPLE_ROS_HHI(){
    for(int i = 0; i < n_devs_; i++){
        delete bluecougars_[i];
    }
};

void BlueCOUGAR_MULTIPLE_ROS_HHI::callbackHHI(const std_msgs::Int32::ConstPtr& msg)
{
    printf("bluecougar callback\n");
    msg_.data = msg->data;
    bool state_grab = true;
    if(msg_.data == 1){ // snapshot grab mode.
        for(int i = 0; i < n_devs_; i++) {
            state_grab = state_grab & bluecougars_[i]->grabImage(img_msgs_[i]);
        }   
        if(state_grab) {
            for(int i = 0; i <n_devs_; i++){
                image_publishers_[i].publish(img_msgs_[i]);
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

void BlueCOUGAR_MULTIPLE_ROS_HHI::callbackDynReconfig(bluecougar::bluecougarDynConfig &config, uint32_t lvl) {
    if(config.hdr){
        for(int i = 0; i < n_devs_; i++)
            bluecougars_[i]->setHighDynamicRange(true);
        cout << " DYNRECONFIG HDR:" <<  config.hdr << "\n";
    }
    else{
        for(int i = 0; i < n_devs_; i++)
            bluecougars_[i]->setHighDynamicRange(false);
    }
        
    if(config.trigger_mode){
        for(int i = 0; i < n_devs_; i++)
            bluecougars_[i]->setTriggerMode(true);
    }
    else{
        for(int i = 0; i < n_devs_; i++)
            bluecougars_[i]->setTriggerMode(false);
    }
    
    if(config.binning_mode){
        for(int i = 0; i < n_devs_; i++)
            bluecougars_[i]->setHardwareBinningMode(true);
    }
    else{
        for(int i = 0; i < n_devs_; i++)
            bluecougars_[i]->setHardwareBinningMode(false);
    }

    if(config.aec){
        for(int i = 0; i < n_devs_; i++){
            bluecougars_[i]->setAutoExposureMode(true);
        }
    }
    else{
        for(int i = 0; i < n_devs_; i++){
            bluecougars_[i]->setAutoExposureMode(false);
            bluecougars_[i]->setExposureTime(config.expose_us);
        }
    }

    if(config.agc){
        for(int i = 0; i < n_devs_; i++)
            bluecougars_[i]->setAutoGainMode(true);
    }
    else{
        for(int i = 0; i < n_devs_; i++)
            bluecougars_[i]->setAutoGainMode(false);
    }

    if(config.wbp == -1){ // off
       for(int i = 0; i < n_devs_; i++)
            bluecougars_[i]->setWhiteBalance(config.wbp,0,0,0);
    }
    else{ // on! each mode...
        for(int i = 0; i < n_devs_; i++)
            bluecougars_[i]->setWhiteBalance(config.wbp,0,0,0);
    }
    
    ROS_INFO("Parameter reconfigured.\n");
};
#endif
