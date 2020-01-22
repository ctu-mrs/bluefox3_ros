#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <iostream>

int main()
{
  DeviceManager devMgr;
  if (devMgr.deviceCount() == 0)
  {
    std::cout << "No devices found!" << std::endl;
    return 1;
  }
  std::cout << "Listing all available devices:" << std::endl;
  
  // show all devices
  for (unsigned int i = 0; i < devMgr.deviceCount(); i++)
    std::cout << "\t" << i << "\t" << (devMgr[i])->deviceID << "\t" << (devMgr[i])->family << "\t" << (devMgr[i])->product << "\t" << (devMgr[i])->serial << std::endl;
  return 0;
}
