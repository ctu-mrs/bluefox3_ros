#include <bluefox3/bluefox3.h>

namespace bluefox3
{

  /* pixelFormatToEncoding() function //{ */
  
  std::string Bluefox3::pixelFormatToEncoding(const PropertyIImageBufferPixelFormat& pixel_format)
  {
    std::string ret = "unknown";
    switch (pixel_format.read())
    {
      case ibpfMono8:
        ret = sensor_msgs::image_encodings::MONO8;
        break;
      case ibpfMono10:
      case ibpfMono12:
      case ibpfMono14:
      case ibpfMono16:
        ret = sensor_msgs::image_encodings::MONO16;
        break;
      case ibpfMono32:
        ret = sensor_msgs::image_encodings::TYPE_32FC1;
        break;
      case ibpfBGR888Packed:
        ret = sensor_msgs::image_encodings::BGR8;
      case ibpfRGB888Packed:
        ret = sensor_msgs::image_encodings::RGB8;
        break;
      case ibpfRGBx888Packed:
        ret = sensor_msgs::image_encodings::RGBA8;
        break;
      case ibpfRGB101010Packed:
      case ibpfRGB121212Packed:
      case ibpfRGB141414Packed:
      case ibpfRGB161616Packed:
        ret = sensor_msgs::image_encodings::RGB16;
        break;
      default:
        ROS_ERROR_STREAM_THROTTLE(1.0, "[" << m_node_name << "]: Unknown pixel format: '" << pixel_format << "'");
        break;
    }
    return ret;
  }
  
  //}

  /* BayerPatternToEncoding() function //{ */
  
  std::string Bluefox3::bayerPatternToEncoding(const PropertyIBayerMosaicParity& bayer_pattern, int bytes_per_pixel)
  {
    if (bytes_per_pixel == 1)
    {
      switch (bayer_pattern)
      {
        case bmpRG:
          return sensor_msgs::image_encodings::BAYER_RGGB8;
        case bmpGB:
          return sensor_msgs::image_encodings::BAYER_GBRG8;
        case bmpGR:
          return sensor_msgs::image_encodings::BAYER_GRBG8;
        case bmpBG:
          return sensor_msgs::image_encodings::BAYER_BGGR8;
        default:
          ROS_ERROR_STREAM_THROTTLE(1.0, "[" << m_node_name << "]: Unknown bayer pattern: '" << bayer_pattern << "'");
      }
    }
    else if (bytes_per_pixel == 2)
    {
      switch (bayer_pattern) {
        case bmpRG:
          return sensor_msgs::image_encodings::BAYER_RGGB16;
        case bmpGB:
          return sensor_msgs::image_encodings::BAYER_GBRG16;
        case bmpGR:
          return sensor_msgs::image_encodings::BAYER_GRBG16;
        case bmpBG:
          return sensor_msgs::image_encodings::BAYER_BGGR16;
        default:
          ROS_ERROR_STREAM_THROTTLE(1.0, "[" << m_node_name << "]: Unknown bayer pattern: '" << bayer_pattern << "'");
      }
    }
    return "unknownBayer";
  }
  
  //}

  /* imageCallback() method //{ */

  void Bluefox3::imageCallback(std::shared_ptr<Request> request_ptr, std::shared_ptr<ThreadParameter> threadParameter_ptr)
  {
    threadParameter_ptr->requestsCaptured++;
    // display some statistical information every 100th image
    if(threadParameter_ptr->requestsCaptured % 100 == 0)
    {
      const Statistics& s = threadParameter_ptr->statistics;
      std::cout << "Info from " << threadParameter_ptr->cameraDevice_ptr->serial.read()
                << ": " << s.framesPerSecond.name() << ": " << s.framesPerSecond.readS()
                << ", " << s.errorCount.name() << ": " << s.errorCount.readS()
                << ", " << s.captureTime_s.name() << ": " << s.captureTime_s.readS() << std::endl;
    }
    if (request_ptr->isOK())
    {
      const auto imw = request_ptr->imageWidth.read();
      const auto imh = request_ptr->imageHeight.read();
      const auto imdata = request_ptr->imageData.read();
      const auto imstep = request_ptr->imageLinePitch.read();

      std::string encoding;
      const auto bayer_mosaic_parity = request_ptr->imageBayerMosaicParity;
      if (bayer_mosaic_parity.read() != bmpUndefined)
      {
        // Bayer pattern
        const auto bytes_per_pixel = request_ptr->imageBytesPerPixel.read();
        encoding = bayerPatternToEncoding(bayer_mosaic_parity, bytes_per_pixel);
      }
      else
      {
        auto encoding = pixelFormatToEncoding(request_ptr->imagePixelFormat);
      }
      sensor_msgs::Image image_msg;
      sensor_msgs::fillImage(image_msg, encoding,
                             imh, imw,
                             imstep,
                             imdata);

      std::scoped_lock lck(m_pub_mtx);
      m_pub.publish(image_msg, cinfoMgr_ptr->getCameraInfo());
    }
    else
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[" << m_node_name.c_str() << "]: Error capturing image: " << request_ptr->requestResult.readS());
    }
  }

  //}

  /* printDevices() method //{ */
  void Bluefox3::printDevices()
  {
    if (m_devMgr.deviceCount() == 0)
    {
      ROS_INFO("[%s]: No devices found!", m_node_name.c_str());
      return;
    }
    ROS_INFO("[%s]: Listing all available devices:", m_node_name.c_str());
    std::cout << "\t#\tID\tfam\tprod\tser" << std::endl;
    // show all devices
    for(unsigned int i = 0; i < m_devMgr.deviceCount(); i++ )
      std::cout << "\t" << i << "\t" << (m_devMgr[i])->deviceID << "\t" << (m_devMgr[i])->family << "\t" << (m_devMgr[i])->product << "\t" << (m_devMgr[i])->serial << std::endl;
  }
  //}

  /* onInit() //{ */

  void Bluefox3::onInit()
  {
    ROS_INFO("[%s]: Initializing", m_node_name.c_str());

    // | ------------------ Begin initialization ------------------ |


    /* initialize ROS node handle etc. //{ */
    
    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
    ros::Time::waitForValid();
    
    //}

    /* load parameters //{ */
    
    mrs_lib::ParamLoader pl(nh, m_node_name);
    
    const auto camera_serial = pl.load_param2<std::string>("camera_serial");
    const auto camera_name = pl.load_param2<std::string>("camera_name");
    const auto calib_url = pl.load_param2<std::string>("calib_url");
    cinfoMgr_ptr = std::make_shared<camera_info_manager::CameraInfoManager>(nh, camera_name, calib_url);
    
    if (!pl.loaded_successfully())
    {
      ROS_ERROR("[%s]: Some compulsory parameters were not loaded successfully, ending the node", m_node_name.c_str());
      ros::shutdown();
    }
    
    //}

    /* initialize publishers //{ */
    
    image_transport::ImageTransport it(nh);
    m_pub = it.advertiseCamera("image_raw", 5);
    
    //}

    /* try to find the camera device //{ */
    
    printDevices();
    m_cameraDevice = m_devMgr.getDeviceBySerial(camera_serial);
    if (m_cameraDevice == nullptr)
    {
      ROS_ERROR("[%s]: Camera with serial '%s' was not found, ending the node", m_node_name.c_str(), camera_serial.c_str());
      ros::shutdown();
    }
    
    //}

      /* try to open the device //{ */
    
      try
      {
        m_cameraDevice->open();
      }
      catch(mvIMPACT::acquire::ImpactAcquireException& e )
      {
        ROS_ERROR("[%s]: An error occurred while opening the device (%s), ending the node", m_node_name.c_str(), e.getErrorString().c_str());
        ros::shutdown();
      }
    
    
      //}

      m_threadParam_ptr = std::make_shared<ThreadParameter>(m_cameraDevice);
      requestProvider_ptr = std::make_shared<helper::RequestProvider>(m_cameraDevice);
      const auto cbk = std::bind(&Bluefox3::imageCallback, this, std::placeholders::_1, std::placeholders::_2);
      requestProvider_ptr->acquisitionStart(cbk, m_threadParam_ptr);

      // | ----------------- Initialization complete ---------------- |

      ROS_INFO("[%s]: Initialized, acquisition started", m_node_name.c_str());
  }
  //}

  Bluefox3::~Bluefox3()
  {
    requestProvider_ptr->acquisitionStop();
  }

}  // namespace bluefox3

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bluefox3::Bluefox3, nodelet::Nodelet)
