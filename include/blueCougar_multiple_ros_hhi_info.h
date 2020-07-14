#ifndef _BLUECOUGAR_MULTIPLE_ROS_HHI_H_
#define _BLUECOUGAR_MULTIPLE_ROS_HHI_H_

#include <iostream>
#include <vector>
#include <sys/time.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>

// for camera info
#include <camera_info_manager/camera_info_manager.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

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

class BlueCOUGAR_MULTIPLE_ROS_HHI {
public:
    explicit BlueCOUGAR_MULTIPLE_ROS_HHI(
        ros::NodeHandle& nh, bool binning_on, bool triggered_on,
        bool aec_on, bool agc_on, int expose_us, double frame_rate)
    : nh_(nh),it_(nh_)
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

            image_transport::CameraPublisher camera_pub_ = it_.advertiseCamera(topic_name,1);
            camera_publishers_.push_back(camera_pub_);

            std::string camera_yaml_name = "file:///home/larrkchlaptop/ros_camera_info/cam" + std::to_string(i) + "_rect.yaml";
            cinfo_mgrs_.push_back(new camera_info_manager::CameraInfoManager(nh_, topic_name, camera_yaml_name));

            img_msgs_.push_back(sensor_msgs::Image());

            // below runs only when two cameras exist.
            if(n_devs_ == 2){   
                undist_maps_x.push_back(cv::Mat::zeros(772,1032,CV_32FC1));
                undist_maps_y.push_back(cv::Mat::zeros(772,1032,CV_32FC1));
            }
        }
        sub_msg_ = nh_.subscribe("/hhi/msg", 1, &BlueCOUGAR_MULTIPLE_ROS_HHI::callbackHHI, this);
        cout << "Please wait for setting cameras...\n";
        ros::Duration(3.0).sleep();
        cout << "camera setting is done.\n";

        if(n_devs_ == 2){

            Eigen::Matrix3d R_0r, R_0l ,R_0n;
            R_0r << 0.995750768633195, -0.00206413024495626, -0.0920659879256872,
                0.00184249567650454, 0.999995196873680 ,-0.00249227591822968,
                0.0920706901032190, 0.00231205447651701 ,0.995749789067522;
            R_0l << 1,0,0,0,1,0,0,0,1;
            R_0n <<0.999167027376312,-0.000660822520147033,-0.0460819845428470,0.000610562206011609,0.999999039521397,-0.00124746416052226,0.0408029241326730,0.00121830615220917,0.998936882157107;

            Eigen::Matrix3d K_l, K_r, K_rect, Kinv_rect;
            K_l << 1205.44081418758, 0, 531.781236677929,0, 1203.44455133544, 404.420699975253,0, 0, 1;
            K_r << 1197.83026641989, 0, 517.468126087747,0,1195.80146146360, 393.186527122459,0, 0, 1;
            K_rect << 1365.49493216333 ,0, 516,0, 1365.49493216333, 386,0, 0 ,1;
            Kinv_rect = K_rect.inverse();

            // interpolation grid calculations.
            float* map_l_x_ptr = nullptr;
            float* map_l_y_ptr = nullptr;
            float* map_r_x_ptr = nullptr;
            float* map_r_y_ptr = nullptr;
            Eigen::Vector3d p_n;
            Eigen::Vector3d P_0, P_l, P_r;
            Eigen::Vector2d p_l, p_r;
            double k1, k2, k3, p1, p2;
            double x, y, r, r2, r4, r6, r_radial, x_dist, y_dist;

            for (int v = 0; v < 772; v++)
            {
                map_l_x_ptr = undist_maps_x[0].ptr<float>(v);
                map_l_y_ptr = undist_maps_y[0].ptr<float>(v);

                map_r_x_ptr = undist_maps_x[1].ptr<float>(v);
                map_r_y_ptr = undist_maps_y[1].ptr<float>(v);

                for (int u = 0; u < 1032; u++)
                {
                    p_n << (double)u, (double)v, 1;
                    P_0 = R_0n*Kinv_rect*p_n;

                    P_l = R_0l.inverse()*P_0;
                    P_l /= P_l(2);

                    P_r = R_0r.inverse()*P_0;
                    P_r /= P_r(2);

                    // left
                    p_l << K_l(0, 0)*P_l(0) + K_l(0, 2),
                    K_l(1, 1)*P_l(1) + K_l(1, 2);
                    k1 = -0.107017421071408;
                    k2 =  0.227783675222806;
                    k3 = 0;
                    p1 = 0;
                    p2 = 0;
                    x = (p_l(0) - K_l(0, 2)) / K_l(0, 0);
                    y = (p_l(1) - K_l(1, 2)) / K_l(1, 1);

                    r = sqrt(x*x + y*y);
                    r2 = r*r;
                    r4 = r2*r2;
                    r6 = r4*r2;

                    r_radial = 1.0 + k1*r2 + k2*r4 + k3*r6;
                    x_dist = x*r_radial + 2 * p1*x*y + p2*(r2 + 2 * x*x);
                    y_dist = y*r_radial + p1*(r2 + 2 * y*y) + 2 * p2*x*y;
                    *(map_l_x_ptr + u) = K_l(0, 2) + x_dist*K_l(0, 0);
                    *(map_l_y_ptr + u) = K_l(1, 2) + y_dist*K_l(1, 1);

                    // right
                    p_r << K_r(0, 0)*P_r(0) + K_r(0, 2),
                    K_r(1, 1)*P_r(1) + K_r(1, 2);
                    k1 = -0.0965384505144884;
                    k2 = 0.207762475094643;
                    k3 = 0;
                    p1 = 0;
                    p2 = 0;

                    x = (p_r(0) - K_r(0, 2)) / K_r(0, 0);
                    y = (p_r(1) - K_r(1, 2)) / K_r(1, 1);
                    r = sqrt(x*x + y*y);
                    r2 = r*r;
                    r4 = r2*r2;
                    r6 = r4*r2;

                    r_radial = 1.0 + k1*r2 + k2*r4 + k3*r6;
                    x_dist = x*r_radial + 2 * p1*x*y + p2*(r2 + 2 * x*x);
                    y_dist = y*r_radial + p1*(r2 + 2 * y*y) + 2 * p2*x*y;
                    *(map_r_x_ptr + u) = K_r(0, 2) + x_dist*K_r(0, 0);
                    *(map_r_y_ptr + u) = K_r(1, 2) + y_dist*K_r(1, 1);
                }
            }
        }
    };    
    ~BlueCOUGAR_MULTIPLE_ROS_HHI();

    void callbackHHI(const std_msgs::Int32::ConstPtr& msg);
    int getStatus() const { return msg_.data;};

private:
    void undistortAndRectify(sensor_msgs::Image &image_msg, const int& cam_id);

    int n_devs_; // # of connected mvBlueCOUGAR cameras.
    mvIMPACT::acquire::DeviceManager devMgr_; // Manager for all devices.

    vector<mvIMPACT::acquire::Device*> validDevices_; // multiple devices
    vector<BlueCougar*> bluecougars_;
    
    
    // For ros.
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    vector<image_transport::CameraPublisher> camera_publishers_;
    vector<camera_info_manager::CameraInfoManager*> cinfo_mgrs_;
    
    vector<sensor_msgs::Image> img_msgs_;

    ros::Subscriber sub_msg_;
    std_msgs::Int32 msg_;

    // for undistortion and rectification
    vector<cv::Mat> undist_maps_x;
    vector<cv::Mat> undist_maps_y;
};

/* IMPLEMENTATION */
BlueCOUGAR_MULTIPLE_ROS_HHI::~BlueCOUGAR_MULTIPLE_ROS_HHI(){
    for(int i = 0; i < n_devs_; i++){
        delete bluecougars_[i];
    }
};
void BlueCOUGAR_MULTIPLE_ROS_HHI::undistortAndRectify(
    sensor_msgs::Image &image_msg, const int& cam_id)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat img_mat = cv_ptr->image;
    cv::remap(img_mat, img_mat, undist_maps_x[cam_id], undist_maps_y[cam_id], CV_INTER_LINEAR);

    std_msgs::Header header; // empty header
    header.seq = 0; // user defined counter
    header.stamp = ros::Time::now(); // time
    cv_bridge::CvImage img_bridge;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_mat);
    img_bridge.toImageMsg(image_msg); 
};

void BlueCOUGAR_MULTIPLE_ROS_HHI::callbackHHI(const std_msgs::Int32::ConstPtr& msg)
{
    msg_.data = msg->data;
    bool state_grab = true;
    if(msg_.data == 1){ // snapshot grab mode.
        for(int i = 0; i < n_devs_; i++){
            state_grab = state_grab & bluecougars_[i]->grabImage(img_msgs_[i]);
        }   
        if(state_grab){
            for(int i = 0; i < n_devs_; i++){       
                // undistortion and rectify images
                if(1){
                    cout << "undistorted!" << endl;
                    undistortAndRectify(img_msgs_[i],i);
                }
                // publish image topics  
                sensor_msgs::CameraInfo camMsg = cinfo_mgrs_[i]->getCameraInfo();
                camera_publishers_[i].publish(img_msgs_[i], camMsg, ros::Time::now());
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
