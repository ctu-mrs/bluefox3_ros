#include <bluefox3/bluefox3.h>

namespace bluefox3
{

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
      std::cout << "Image captured: " << request_ptr->imageOffsetX.read() << "x" << request_ptr->imageOffsetY.read() << "@" << request_ptr->imageWidth.read() << "x" << request_ptr->imageHeight.read() << std::endl;
    }
    else
    {
      std::cout << "Error: " << request_ptr->requestResult.readS() << std::endl;
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
    
    const std::string camera_serial = pl.load_param2<std::string>("camera_serial");
    
    if (!pl.loaded_successfully())
    {
      ROS_ERROR("[%s]: Some compulsory parameters were not loaded successfully, ending the node", m_node_name.c_str());
      ros::shutdown();
    }
    
    //}

    /* initialize publishers //{ */
    
    image_transport::ImageTransport it(nh);
    m_pub_output = it.advertise("image_raw", 5);
    
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
