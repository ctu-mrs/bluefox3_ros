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

  /* bayerPatternToEncoding() function //{ */

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
    } else if (bytes_per_pixel == 2)
    {
      switch (bayer_pattern)
      {
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
    const ros::Time cbk_time = ros::Time::now();
    threadParameter_ptr->requestsCaptured++;
    // display some statistical information every 100th image
    const Statistics& s = threadParameter_ptr->statistics;
    if (threadParameter_ptr->requestsCaptured % 50 == 0)
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "[" << m_node_name.c_str() << "]: "
                                        << "Info from " << threadParameter_ptr->cameraDevice_ptr->serial.read() << ": " << s.framesPerSecond.name() << ": "
                                        << s.framesPerSecond.readS() << ", " << s.errorCount.name() << ": " << s.errorCount.readS() << ", "
                                        << s.captureTime_s.name() << ": " << s.captureTime_s.readS());
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
      } else
      {
        encoding = pixelFormatToEncoding(request_ptr->imagePixelFormat);
      }
      sensor_msgs::Image image_msg;
      sensor_msgs::fillImage(image_msg, encoding, imh, imw, imstep, imdata);
      const ros::Time stamp = cbk_time - ros::Duration(s.captureTime_s.read());
      image_msg.header.stamp = stamp;
      image_msg.header.frame_id = m_frame_id;

      sensor_msgs::CameraInfo cinfo_msg = m_cinfoMgr_ptr->getCameraInfo();
      cinfo_msg.header = image_msg.header;

      std::scoped_lock lck(m_pub_mtx);
      m_pub.publish(image_msg, cinfo_msg);
    } else
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
    // show all devices
    for (unsigned int i = 0; i < m_devMgr.deviceCount(); i++)
      std::cout << "\t" << i << "\t" << (m_devMgr[i])->deviceID << "\t" << (m_devMgr[i])->family << "\t" << (m_devMgr[i])->product << "\t"
                << (m_devMgr[i])->serial << std::endl;
  }
  //}

  /* convenience functions for property reading/writing //{ */
  template <typename PropertyType, typename ValueType>
  ValueType clampProperty(const PropertyType& prop, const ValueType& value)
  {
    return std::clamp(value, prop.getMinValue(), prop.getMaxValue());
  }

  template <typename ValueType>
  using TranslationDict = std::vector<std::pair<std::string, ValueType>>;

  template <typename PropertyType>
  TranslationDict<typename PropertyType::value_type> getTranslationDict(const PropertyType& prop)
  {
    TranslationDict<typename PropertyType::value_type> dict;
    prop.getTranslationDict(dict);
    return dict;
  }

  template <typename ValueType>
  void printTranslationDict(const TranslationDict<ValueType>& dict)
  {
    for (const auto& p : dict)
    {
      std::cout << "[" << p.first << ": " << p.second << "]";
    }
    std::cout << std::endl;
  }
  //}

  /* writeProperty() method //{ */
  template <typename PropertyType, typename ValueType>
  void Bluefox3::writeProperty(const PropertyType& prop, ValueType value)
  {
    using PropertyValueType = typename PropertyType::value_type;
    // Check if it's possible to write to this property
    if (!(prop.isVisible() && prop.isWriteable() && prop.isValid()))
    {
      ROS_ERROR_STREAM("[" << m_node_name << "]: " << prop.name() << ": unable to write property");
      return;
    }

    // Clamp value to valid range only if it's not an enum
    if (!std::is_enum<PropertyValueType>::value)
    {
      if (prop.hasMaxValue() && prop.hasMinValue())
      {
        value = clampProperty(prop, value);
      }
    }

    try
    {
      prop.write(static_cast<PropertyValueType>(value));
    }
    catch (...)
    {
      ROS_ERROR_STREAM("[" << m_node_name << "]: " << prop.name() << ": failed to write property");
      printTranslationDict(getTranslationDict(prop));
    }
  }
  //}

  /* readProperty() method //{ */
  template <typename PropertyType, typename ValueType>
  void Bluefox3::readProperty(const PropertyType& prop, ValueType& value)
  {
    if (!(prop.isValid() && prop.isVisible()))
    {
      ROS_ERROR_STREAM("[" << m_node_name << "]: " << prop.name() << ": unable to read property");
      return;
    }

    try
    {
      value = static_cast<ValueType>(prop.read());
    }
    catch (...)
    {
      ROS_ERROR_STREAM("[" << m_node_name << "]: " << prop.name() << ": failed to read property");
    }
  }
  //}

  /* writeAndReadProperty() method //{ */
  template <typename PropertyType, typename ValueType>
  void writeAndReadProperty(const PropertyType& prop, ValueType& value)
  {
    writeProperty(prop, value);
    readProperty(prop, value);
  }
  //}

  /* setMirrorMode() method //{ */
  static const std::map<std::string, TMirrorMode> str2mm =
  {
    {"mmOff",	                mmOff},
    {"mmTopDown",	            mmTopDown},
    {"mmLeftRight",	          mmLeftRight},
    {"mmTopDownAndLeftRight", mmTopDownAndLeftRight},
  };

  void Bluefox3::setMirrorMode(const TMirrorMode mirror_mode)
  {
    writeProperty(m_imgProc_ptr->mirrorModeGlobal, mirror_mode);
  }
  //}

  /* setWhiteBalance() method //{ */
  static const std::map<std::string, TWhiteBalanceParameter> str2wbp =
  {
    {"wbpTungsten",	    wbpTungsten},
    {"wbpHalogen",	    wbpHalogen},
    {"wbpFluorescent",	wbpFluorescent},
    {"wbpDayLight",	    wbpDayLight},
    {"wbpPhotoFlash",	  wbpPhotoFlash},
    {"wbpBlueSky",	    wbpBlueSky},
    {"wbpUser1",	      wbpUser1},
    {"wbpUser2",	      wbpUser2},
    {"wbpUser3",	      wbpUser3},
    {"wbpUser4",	      wbpUser4},
  };

  int usrWhiteBalanceNumber(const TWhiteBalanceParameter wbp)
  {
    switch (wbp)
    {
      case wbpUser1:
        return 0;
      case wbpUser2:
        return 1;
      case wbpUser3:
        return 2;
      case wbpUser4:
        return 3;
      default:
        return -1;
    }
  }

  void Bluefox3::setWhiteBalance(const TWhiteBalanceParameter wbp, const double r_gain, const double g_gain, const double b_gain)
  {
    // Put white balance as unavailable if it's not a color camera
    if (m_bfxInfo_ptr->sensorColorMode.read() <= iscmMono)
      return;

    switch (wbp)
    {
      case wbpTungsten:
      case wbpHalogen:
      case wbpFluorescent:
      case wbpDayLight:
      case wbpPhotoFlash:
      case wbpBlueSky:
        writeProperty(m_imgProc_ptr->whiteBalance, wbp);
        break;
      default:
      {
        const auto usr_n = usrWhiteBalanceNumber(wbp);
        if (usr_n < 0)
        {
          ROS_ERROR("[%s]: Unknown white balance setting: %d!", m_node_name.c_str(), (int)wbp);
          return;
        }
        writeProperty(m_imgProc_ptr->whiteBalance, wbp);
        auto wbp_set = m_imgProc_ptr->getWBUserSetting(0);
        writeAndReadProperty(wbp_set.redGain, r_gain);
        writeAndReadProperty(wbp_set.greenGain, g_gain);
        writeAndReadProperty(wbp_set.blueGain, b_gain);
      }
      break;
    }
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
    m_cinfoMgr_ptr = std::make_shared<camera_info_manager::CameraInfoManager>(nh, camera_name, calib_url);
    m_frame_id = pl.load_param2<std::string>("frame_id");

    const std::string imgproc_mirror_mode_name = pl.load_param2<std::string>("imgproc/mirror_mode");
    const std::string imgproc_white_balance_mode_name = pl.load_param2<std::string>("imgproc/white_balance_mode");
    const double imgproc_white_balance_gainr = pl.load_param2<double>("imgproc/white_balance/gain/r");
    const double imgproc_white_balance_gaing = pl.load_param2<double>("imgproc/white_balance/gain/g");
    const double imgproc_white_balance_gainb = pl.load_param2<double>("imgproc/white_balance/gain/b");

    if (!pl.loaded_successfully())
    {
      ROS_ERROR("[%s]: Some compulsory parameters were not loaded successfully, ending the node", m_node_name.c_str());
      ros::shutdown();
    }

    if (!str2mm.count(imgproc_mirror_mode_name))
    {
      ROS_ERROR("[%s]: Invalid mirror mode selected! Valid values are:", m_node_name.c_str());
      for (const auto& kv : str2mm)
        std::cout << "\t" << kv.first << std::endl;
      ros::shutdown();
    }
    const TMirrorMode imgproc_mirror_mode = str2mm.at(imgproc_mirror_mode_name);

    if (!str2wbp.count(imgproc_white_balance_mode_name))
    {
      ROS_ERROR("[%s]: Invalid white balance mode selected! Valid values are:", m_node_name.c_str());
      for (const auto& kv : str2wbp)
        std::cout << "\t" << kv.first << std::endl;
      ros::shutdown();
    }
    const TWhiteBalanceParameter imgproc_white_balance_mode = str2wbp.at(imgproc_white_balance_mode_name);

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
    catch (mvIMPACT::acquire::ImpactAcquireException& e)
    {
      ROS_ERROR("[%s]: An error occurred while opening the device (%s), ending the node", m_node_name.c_str(), e.getErrorString().c_str());
      ros::shutdown();
    }


    //}

    m_bfxInfo_ptr = std::make_shared<InfoBlueDevice>(m_cameraDevice);
    m_imgProc_ptr = std::make_shared<ImageProcessing>(m_cameraDevice);
    m_threadParam_ptr = std::make_shared<ThreadParameter>(m_cameraDevice);
    requestProvider_ptr = std::make_shared<helper::RequestProvider>(m_cameraDevice);

    setMirrorMode(imgproc_mirror_mode);
    setWhiteBalance(imgproc_white_balance_mode, imgproc_white_balance_gainr, imgproc_white_balance_gaing, imgproc_white_balance_gainb);

    // | ----------- Start the actual image acquisition ----------- |
    const auto cbk = std::bind(&Bluefox3::imageCallback, this, std::placeholders::_1, std::placeholders::_2);
    requestProvider_ptr->acquisitionStart(cbk, m_threadParam_ptr);

    // | ----------------- Initialization complete ---------------- |

    ROS_INFO("[%s]: Initialized, acquisition started", m_node_name.c_str());
  }
  //}

  /* Bluefox3() method //{ */
  Bluefox3::~Bluefox3()
  {
    requestProvider_ptr->acquisitionStop();
  }
  //}

}  // namespace bluefox3

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bluefox3::Bluefox3, nodelet::Nodelet)
