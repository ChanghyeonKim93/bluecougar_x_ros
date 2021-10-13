#ifndef _BLUECOUGAR_H_
#define _BLUECOUGAR_H_

#include <iostream>
#include <vector>
#include <sys/time.h>

#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>

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

using namespace std;
using namespace mvIMPACT::acquire;
using namespace mvIMPACT::acquire::GenICam;
using namespace sensor_msgs::image_encodings;

std::mutex s_mutex;


/**
 * @brief PixelFormatToEncoding Convert pixel format to image encoding
 * @param pixel_format mvIMPACT ImageBufferPixelFormat
 * @return Image encoding
 */
std::string PixelFormatToEncoding(const TImageBufferPixelFormat& pixel_format) {
  switch (pixel_format) {
    case ibpfMono8:
      return MONO8;
    case ibpfMono16:
      return MONO16;
    case ibpfRGBx888Packed:
      return BGRA8;
    case ibpfRGB888Packed:
      return BGR8;
    case ibpfBGR888Packed:
      return RGB8;
    case ibpfRGB161616Packed:
      return BGR16;
    default:
      return MONO8;
  }
};
/**
 * @brief BayerPatternToEncoding Convert bayer pattern to image encoding
 * @param bayer_pattern mvIMPACT BayerMosaicParity
 * @param bytes_per_pixel Number of bytes per pixel
 * @return Image encoding
 */
std::string BayerPatternToEncoding(const TBayerMosaicParity& bayer_pattern,
                                   int bytes_per_pixel) {
  if (bytes_per_pixel == 1) {
    switch (bayer_pattern) {
      case bmpRG:
        return BAYER_RGGB8;
      case bmpGB:
        return BAYER_GBRG8;
      case bmpGR:
        return BAYER_GRBG8;
      case bmpBG:
        return BAYER_BGGR8;
      default:
        return MONO8;
    }
  } else if (bytes_per_pixel == 2) {
    switch (bayer_pattern) {
      case bmpRG:
        return BAYER_RGGB16;
      case bmpGB:
        return BAYER_GBRG16;
      case bmpGR:
        return BAYER_GRBG16;
      case bmpBG:
        return BAYER_BGGR16;
      default:
        return MONO16;
    }
  }
  return MONO8;
};

//-----------------------------------------------------------------------------
class BlueCougar;
class ThreadData;

class ThreadData {
    volatile bool is_terminated_;
    unique_ptr<thread> p_thread_;
public:
    explicit ThreadData() : is_terminated_( false ), p_thread_( nullptr ) {};
    virtual ~ThreadData() {};

    bool isTerminated() const { return is_terminated_; };
    
    template<class _Fn, class _Arg>
    void startThread( _Fn&& _Fx, _Arg&& _Ax ) {
        p_thread_ = unique_ptr<thread>( new thread( _Fx, _Ax ) );
    };
    void terminateThread() {
        is_terminated_ = true;
        if( p_thread_ != nullptr ) {
            p_thread_->join();
            cout << "A thread join! \n";
        }
    };
};


class BlueCougar : public ThreadData {
  public:
    BlueCougar(mvIMPACT::acquire::Device* dev, int cam_id, 
      bool binning_on, bool triggered_on, bool aec_on, bool agc_on, 
      int expose_us, double frame_rate);
    ~BlueCougar();
    bool grabImage(sensor_msgs::Image &image_msg);
    bool grabImageThread(sensor_msgs::Image &image_msg);
    void setHardwareTriggeredSnapshotMode();

    void setTriggerMode(bool onoff);
    void setHardwareBinningMode(bool onoff);
    void setSoftwareBinningMode(bool onoff, int lvl);
    void getSoftwareBinning(int lvl, uint8_t* src, uint8_t* dst);
    void setExposureTime(const int& expose_us);
    void setGain(const int& gain);
    void setFrameRate(const int& frame_rate);
    void setAutoExposureMode(bool onoff);
    void setAutoGainMode(bool onoff);
    void setWhiteBalance(const int& wbp_mode, double r_gain, double g_gain, double b_gain);

    void setHighDynamicRange(bool hdr_onoff);

    inline double getExposureTime(){return cs_->expose_us.read();};
    inline double getGain(){return cs_->gain_dB.read();};
    inline double getFrameRate(){return cs_->frameRate_Hz.read();};

   // getters
    mvIMPACT::acquire::Device*  device() const {return dev_; };
    mvIMPACT::acquire::Request* request() {return request_; };
    mvIMPACT::acquire::FunctionInterface* functioninterface() const { return fi_; };
    mvIMPACT::acquire::ImageProcessing* imageprocessing() const {return img_proc_; };
    mvIMPACT::acquire::CameraSettingsBlueCOUGAR* camerasettingbluecougar() const {return cs_; };
    mvIMPACT::acquire::Statistics* statistics() const { return stat_; }; 
    bool isBinningOn() const {return binning_on_;};
    bool isTriggerOn() const {return trigger_on_;};
    bool isAecOn() const {return aec_on_; };
    bool isAgcOn() const {return agc_on_; };
    int exposeus() const {return expose_us_;};
    double framerate() const {return frame_rate_;};
    string serial() const {return serial_; };
    string frameid() const {return frame_id_;};
    int cntimg() const {return cnt_img;};
    void addCntImg() { ++cnt_img;};

    void setGrabbed() {is_grabbed_ = true;};
    void setUnGrabbed() {is_grabbed_ = false;};

    int curr_request_nr_;

  private:
    bool binning_on_;
    bool trigger_on_; 
    bool aec_on_;
    bool agc_on_;
    int expose_us_;
    double frame_rate_;
    int cnt_img;
    string serial_;
    string frame_id_;

    bool is_grabbed_;


    bool software_binning_on_;
    bool software_binning_level_;


    mvIMPACT::acquire::DeviceManager devMgr_; // not used in single.
    mvIMPACT::acquire::Device* dev_{nullptr}; // multiple devices
    mvIMPACT::acquire::Request *request_{nullptr};
    mvIMPACT::acquire::FunctionInterface *fi_{nullptr};
    mvIMPACT::acquire::ImageProcessing *img_proc_{nullptr};
    mvIMPACT::acquire::CameraSettingsBlueCOUGAR *cs_{nullptr};
    mvIMPACT::acquire::Statistics *stat_{nullptr};
};


void liveThread(BlueCougar* bluecougar){
  {
    lock_guard<mutex> lockedScope( s_mutex );
    cout << "[BlueCOUGAR THREAD] start thread for [" << bluecougar->serial() << "]\n";
  }
    
  // establish access to the statistic properties
  Statistics* pSS        = bluecougar->statistics();
  FunctionInterface* pFI = bluecougar->functioninterface();
  Request* pREQUEST      = bluecougar->request();

  const unsigned int timeout_ms = {200};
  // run thread loop
  while( !bluecougar->isTerminated() ) {
    int error_msg = pFI->imageRequestSingle();
    if(error_msg == mvIMPACT::acquire::DEV_NO_FREE_REQUEST_AVAILABLE){
      //lock_guard<mutex> lockedScope( s_mutex );
      //std::cout<<"[BlueCOUGAR THREAD info] Cam [" << bluecougar->frameid() << "]: the camera is not available...\n";
      continue;
    }
      
    // wait for results from the default capture queue
    bluecougar->curr_request_nr_ = pFI->imageRequestWaitFor( timeout_ms );

    if(!pFI->isRequestNrValid( bluecougar->curr_request_nr_ ) ) {
      continue;
    }
    pREQUEST = pFI->getRequest( bluecougar->curr_request_nr_ );
    if( !pREQUEST->isOK() ) {
      lock_guard<mutex> lockedScope( s_mutex );
      cout << "[BlueCOUGAR THREAD info] Cam ["<< bluecougar->frameid() << "]: fail to rcv..."
           << " Error message: " << pREQUEST->requestResult.readS() <<"\n";
      bluecougar->setUnGrabbed();
      continue;
    }  
    {
      lock_guard<mutex> lockedScope( s_mutex );
      bluecougar->setGrabbed();
      bluecougar->addCntImg();
      
      cout << "[BlueCOUGAR THREAD info] from " << bluecougar->device()->serial.read()
            << ": " << pSS->framesPerSecond.name() << ": " << pSS->framesPerSecond.readS()
            << ", " << pSS->errorCount.name() << ": " << pSS->errorCount.readS()
            << ", " << pREQUEST->infoFrameNr
            << ", " << pREQUEST->infoFrameID
            << ", " << pSS->frameCount << endl;
    }
  }
};


/* IMPLEMENTATION */
BlueCougar::BlueCougar(mvIMPACT::acquire::Device* dev, int cam_id, bool binning_on, 
bool trigger_on, bool aec_on, bool agc_on, int expose_us, double frame_rate) 
: dev_(dev), binning_on_(binning_on), trigger_on_(trigger_on), aec_on_(aec_on), 
agc_on_(agc_on), expose_us_(expose_us), frame_rate_(frame_rate), is_grabbed_(false),
software_binning_on_(false), software_binning_level_(false)
{
    cnt_img = 0;
    
    dev_->open();
    frame_id_ = std::to_string(cam_id);
    serial_   = dev_->serial.read();
    cout << "Type: " << dev_->product.read() << " / serial[" << serial_ << "]";
    cs_   = new mvIMPACT::acquire::CameraSettingsBlueCOUGAR(dev_);
    fi_   = new mvIMPACT::acquire::FunctionInterface(dev_);
    stat_ = new mvIMPACT::acquire::Statistics(dev_);
    img_proc_= new mvIMPACT::acquire::ImageProcessing(dev_); // for White balance

    cs_->binningMode.write(cbmOff);
    //cs_->autoControlMode.write(acmStandard);
    //cs_->triggerMode.write(ctmContinuous); // ctmOnDemand ctmContinuous
    if(binning_on_ == true) cs_->binningMode.write(cbmBinningHV); // cbmOff: no binning. / cbmBinningHV
    else cs_->binningMode.write(cbmOff); // no binning 

    cs_->expose_us.write(expose_us_);
    cs_->frameRate_Hz.write(frame_rate_);

    if(aec_on_ == true) cs_->autoExposeControl.write(aecOn); // auto expose ?
    if(agc_on_ == true) cs_->autoGainControl.write(agcOn); // auto gain ?

    cout << " / exp_ctrl: "<<cs_->autoExposeControl.read();
    cout << " / freq: "<<cs_->frameRate_Hz.read()<<" [Hz]" << endl;

    if(trigger_on_ == true) setHardwareTriggeredSnapshotMode();
    cout << " ============= \n";
};

BlueCougar::~BlueCougar() {
  if (dev_ && dev_->isOpen()) {
    delete cs_;
    delete fi_;
    delete stat_;
    dev_->close();
  }
};

void BlueCougar::setHardwareTriggeredSnapshotMode() {
  cout<<"[BlueCOUGAR info] Set ["<<serial_<<"] in trigger mode."<<endl;
    // trigger mode
    // ctsDigIn0 : digitalInput 0 as trigger source
    // In this application an image is triggered by a rising edge. (over +3.3 V) 
    cs_->triggerSource.write(ctsDigIn0);
    cs_->triggerMode.write(ctmOnRisingEdge);
    //cs_->imageRequestTimeout_ms.write(0); // infinite trigger timeout
   
    //cs_->autoExposeControl.write(aecOff); // auto expose ?
    //cs_->autoGainControl.write(agcOff); // auto gain ?
    //cs_->exposeMode.write(cemStandard);
    //cs_->binningMode.write(cbmOff); // cbmOff: no binning. 
    //cs_->expose_us.write(10000);
    // cbmBinningHV: half resolution binning.

    cout<<"[BlueCOUGAR info]   trigger source: "<<cs_->triggerSource.read();
    cout<<" / trigger mode: "<<cs_->triggerMode.read();
    cout<<" / exposure time: "<<cs_->expose_us.read()<< "[us]" << endl;
};

void BlueCougar::setExposureTime(const int& expose_us){
  cs_->expose_us.write(expose_us);
  std::cout<<"[BlueCOUGAR info] set exposure time: "<<cs_->expose_us.read()<< "[us]"<<std::endl;
};
void BlueCougar::setFrameRate(const int& frame_rate){
  cs_->frameRate_Hz.write(frame_rate);
};


bool BlueCougar::grabImage(sensor_msgs::Image &image_msg){
  // NOTE: A request object is locked for the driver whenever the corresponding
  // wait function returns a valid request object.
  // All requests returned by
  // mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor need to be
  // unlocked no matter which result mvIMPACT::acquire::Request::requestResult
  // contains.
  // http://www.matrix-vision.com/manuals/SDK_CPP/ImageAcquisition_section_capture.html
    std::cout << "try to recv cam [" << frame_id_ <<"]\n";
    int error_msg = fi_->imageRequestSingle();
    if(error_msg == mvIMPACT::acquire::DEV_NO_FREE_REQUEST_AVAILABLE) 
      std::cout<<"Cam [" << frame_id_ << "]: the camera is not available...\n";

    int request_nr = fi_->imageRequestWaitFor(50);
    // if failed,
    if(!fi_->isRequestNrValid( request_nr )) {
        return false;
    }

    request_ = fi_->getRequest(request_nr);
    // Check if request is ok
    if (!request_->isOK()) {
        // need to unlock here because the request is valid even if it is not ok
        std::cout<< "[BlueCOUGAR info] Cam ["<< frame_id_<< "]: fail to rcv... \n";
        return false;
    }
    ++cnt_img;
    std::cout<< "[BlueCOUGAR info] Cam ["<< frame_id_<< "]: rcv success! # of img [" << cnt_img <<"]";

    std::string encoding;
    const auto bayer_mosaic_parity = request_->imageBayerMosaicParity.read();
    if (bayer_mosaic_parity != bmpUndefined) {
        // Bayer pattern
        const auto bytes_per_pixel = request_->imageBytesPerPixel.read();
        encoding = BayerPatternToEncoding(bayer_mosaic_parity, bytes_per_pixel);
    } else {
        encoding = PixelFormatToEncoding(request_->imagePixelFormat.read());
    }
    image_msg.header.frame_id = frame_id_;
    sensor_msgs::fillImage(image_msg, encoding, request_->imageHeight.read(),
                         request_->imageWidth.read(),
                         request_->imageLinePitch.read(),
                         request_->imageData.read());
    std::cout<<" sz: [" << request_->imageWidth.read()
    << "x" << request_->imageHeight.read()<<"]\n";
    fi_->imageRequestUnlock(request_nr);
    return true;
};

void BlueCougar::setHardwareBinningMode(bool onoff){
  if(!software_binning_on_){
    if(onoff == true) {
      cs_->binningMode.write(cbmBinningHV); // cbmBinningHV
      binning_on_ = true;
    }
    else {
      cs_->binningMode.write(cbmOff); // cbmOff: no binning. 
      binning_on_ = false;
    }
  }
  else{
    cs_->binningMode.write(cbmOff); 
    binning_on_ = false;
    cout <<" WARN: software binning mode intervenes.\n";
  }
};

void BlueCougar::setSoftwareBinningMode(bool onoff, int lvl){
  if(onoff == true) { // software on
    cs_->binningMode.write(cbmOff);
    software_binning_level_ = lvl;
    software_binning_on_ = true;
  }else{
    software_binning_on_ = false;
  }
}

void BlueCougar::getSoftwareBinning(int lvl, uint8_t* src, uint8_t* dst){
  int index = 0;
  int den = std::pow(2,lvl);
  int stepsz_dst = 3008/den;
  int stepsz_org = 3008*den;
  
  int den4 = den*4;
  for(int v = 0; v < 480/den; v++){
    for(int u = 0; u < 752/den; u++){
      int u4 = 4*u;
      *(dst+v*stepsz_dst+u4)   = *(src+v*stepsz_org+u*den4);	
      *(dst+v*stepsz_dst+u4+1) = *(src+v*stepsz_org+u*den4+1);	    
      *(dst+v*stepsz_dst+u4+2) = *(src+v*stepsz_org+u*den4+2);	    
      *(dst+v*stepsz_dst+u4+3) = *(src+v*stepsz_org+u*den4+3);	        
    }
  }
}

void BlueCougar::setTriggerMode(bool onoff) {
  if(onoff == true){
    cout<<"Set ["<<serial_<<"] in trigger mode."<<endl;
    // trigger mode
    // ctsDigIn0 : digitalInput 0 as trigger source
    // In this application an image is triggered by a rising edge. (over +3.3 V) 
    cs_->triggerSource.write(ctsDigIn0);
    cs_->triggerMode.write(ctmOnRisingEdge); // ctmOnRisingEdge ctmOnHighLevel
    cs_->frameDelay_us.write(0);
    cout<<"  trigger source: "<<cs_->triggerSource.read();
    cout<<" / trigger mode: "<<cs_->triggerMode.read();
    cout<<" / exposure time: "<<cs_->expose_us.read()<< "[us]" << endl;
  }
  else{
    cs_->triggerMode.write(ctmContinuous);
  }
};

void BlueCougar::setAutoExposureMode(bool onoff){
  if(onoff) cs_->autoExposeControl.write(aecOn);
  else cs_->autoExposeControl.write(aecOff);
};
void BlueCougar::setAutoGainMode(bool onoff){
  if(onoff) cs_->autoGainControl.write(agcOn);
  else cs_->autoGainControl.write(agcOff);
};

void BlueCougar::setGain(const int& gain){
  cs_->gain_dB.write(gain);
};
void BlueCougar::setWhiteBalance(const int& wbp_mode, double r_gain, double g_gain, double b_gain){
  // white balance
    // user defined white balance parameters
    // wbpTungsten, wbpHalogen, wbpFluorescent, wbpDayLight, wbpPhotoFlash, wbpBlueSky, wbpUser1.

  if(wbp_mode == -1){
  }
  else if(wbp_mode == wbpTungsten){
    img_proc_->whiteBalance.write(wbpTungsten);
  }
  else if(wbp_mode == wbpHalogen){
    img_proc_->whiteBalance.write(wbpHalogen);
  }
  else if(wbp_mode == wbpFluorescent){
    img_proc_->whiteBalance.write(wbpFluorescent);
  }
  else if(wbp_mode == wbpDayLight){
    img_proc_->whiteBalance.write(wbpDayLight);
  }
  else if(wbp_mode == wbpPhotoFlash){
    img_proc_->whiteBalance.write(wbpPhotoFlash);
  }
  else if(wbp_mode == wbpBlueSky){
    img_proc_->whiteBalance.write(wbpBlueSky);
  }
  else if(wbp_mode == wbpUser1){
    auto wbp_set = img_proc_->getWBUserSetting(0);
    wbp_set.redGain.write(r_gain);
    wbp_set.greenGain.write(g_gain);
    wbp_set.blueGain.write(b_gain);
  }
};

void BlueCougar::setHighDynamicRange(bool hdr_onoff){
  //set HDR mode
  auto &hdr_control = cs_->getHDRControl();
  if(!hdr_control.isAvailable()){
    hdr_onoff = false;
    cout << " HDR control is not supported.\n";
    return;
  }
  // cHDRmFixed0,cHDRmFixed1,cHDRmFixed2,cHDRmFixed3,cHDRmFixed4,cHDRmFixed5,cHDRmFixed6,cHDRmUser
  if(hdr_onoff){
    hdr_control.HDREnable.write(bTrue); // set HDR on/off.
    hdr_control.HDRMode.write(cHDRmFixed0);
  }
  else{
    hdr_control.HDREnable.write(bFalse); // set HDR on/off.
  }
};


bool BlueCougar::grabImageThread(sensor_msgs::Image &image_msg){
  if(is_grabbed_){
    lock_guard<mutex> lockedScope( s_mutex );
    std::cout<< "[BlueCOUGAR info] Cam ["<< this->frame_id_<< "]: rcv success! # of img [" << cnt_img <<"] ";

    std::string encoding;
    const auto bayer_mosaic_parity = request_->imageBayerMosaicParity.read();
    if (bayer_mosaic_parity != bmpUndefined) {
        // Bayer pattern
        const auto bytes_per_pixel = request_->imageBytesPerPixel.read();
        encoding = BayerPatternToEncoding(bayer_mosaic_parity, bytes_per_pixel);
    } else {
        encoding = PixelFormatToEncoding(request_->imagePixelFormat.read());
    }
    image_msg.header.frame_id = frame_id_;
    sensor_msgs::fillImage(image_msg, encoding, request_->imageHeight.read(),
                         request_->imageWidth.read(),
                         request_->imageLinePitch.read(),
                         request_->imageData.read());
    std::cout<<" sz: [" << request_->imageWidth.read()
    << "x" << request_->imageHeight.read()<<"]";

    // Unlock the used request
    fi_->imageRequestUnlock(curr_request_nr_);

    // initialize grab status
    this->setUnGrabbed();
    return true;
  }
  else{
    // initialize grab status
    lock_guard<mutex> lockedScope( s_mutex );
    this->setUnGrabbed();
    return false;
  }
    
};

#endif
