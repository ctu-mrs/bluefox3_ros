#ifndef BLUEFOX3_H
#define BLUEFOX3_H

/* includes //{ */

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>

// Msgs
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// Bluefox stuff
#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_helper.h>
#include <mvDeviceManager/Include/mvDeviceManager.h>

// std
#include <mutex>
#include <functional>

// local stuff
#include <bluefox3/Bluefox3Config.h>

//}

namespace bluefox3
{

  /* struct ThreadParameter //{ */

  struct ThreadParameter
  {
    Device* cameraDevice_ptr;
    unsigned int requestsCaptured;
    Statistics statistics;
    explicit ThreadParameter(Device* cameraDevice_ptr) : cameraDevice_ptr(cameraDevice_ptr), requestsCaptured(0), statistics(cameraDevice_ptr)
    {
    }
    ThreadParameter(const ThreadParameter& src) = delete;
    ThreadParameter& operator=(const ThreadParameter& rhs) = delete;
  };

  //}

  /* class Bluefox3 //{ */

  class Bluefox3 : public nodelet::Nodelet
  {
  public:
    Bluefox3() : m_node_name("Bluefox3"), m_running(false) {};
    ~Bluefox3();
    virtual void onInit();
    void printDevices();

  private:
    const std::string m_node_name;
    bool m_running;

  private:
    DeviceManager m_devMgr;
    Device* m_cameraDevice;
    std::shared_ptr<GenICam::AcquisitionControl> m_GenICamACQ_ptr;
    std::shared_ptr<ImageProcessing> m_imgProc_ptr;
    std::shared_ptr<ThreadParameter> m_threadParam_ptr;
    std::shared_ptr<helper::RequestProvider> requestProvider_ptr;

  private:
    std::mutex m_pub_mtx;
    image_transport::CameraPublisher m_pub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> m_cinfoMgr_ptr;

  private:
    std::string pixelFormatToEncoding(const PropertyIImageBufferPixelFormat& pixel_format);
    std::string bayerPatternToEncoding(const PropertyIBayerMosaicParity& bayer_pattern, int bytes_per_pixel);

    // | -------------- Property manipulation methods ------------- |
    template <typename PropertyType>
    void writeProperty(const PropertyType& prop, typename PropertyType::value_type value);
    template <typename PropertyType>
    bool readProperty(const PropertyType& prop, typename PropertyType::value_type& value);
    template <typename PropertyType>
    void writeAndReadProperty(const PropertyType& prop, typename PropertyType::value_type& value);
    template <typename PropertyType>
    void writeDictProperty(const PropertyType& prop, const std::string& keystr);
    template <typename PropertyType>
    bool readDictProperty(const PropertyType& prop, std::string& key);
    template <typename PropertyType>
    void writeAndReadDictProperty(const PropertyType& prop, std::string& value);

  private:
    template <typename PropertyType>
    void setMirrorMode(const PropertyType& prop, bool& TopDown, bool& LeftRight);
    /* void setWhiteBalance(const TWhiteBalanceParameter wbp, const double r_gain, const double g_gain, const double b_gain); */

  private:
    // | --------------------- Helper methods --------------------- |
    void imageCallback(std::shared_ptr<Request> pRequest, std::shared_ptr<ThreadParameter> threadParameter_ptr);

    void dynRecCallback(bluefox3::Bluefox3Config& cfg, [[maybe_unused]] uint32_t level);

    template <typename T>
    bool getParamCheck(const ros::NodeHandle& nh, const std::string& param_name, T& param_out);

    template <typename T>
    bool getParamCheck(const ros::NodeHandle& nh, const std::string& param_name, T& param_out, const T& param_default);

  private:
    // --------------------------------------------------------------
    // |                ROS-related member variables                |
    // --------------------------------------------------------------

    /* Parameters, loaded from ROS //{ */

    std::string m_frame_id;

    //}

    boost::recursive_mutex m_dynRecServer_mtx;
    std::shared_ptr<dynamic_reconfigure::Server<Bluefox3Config>> m_dynRecServer_ptr;
    Bluefox3Config m_lastCfg;
  };

  //}

}  // namespace bluefox3

#endif  // #ifndef BALLOONFILTER
