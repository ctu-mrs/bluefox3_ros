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
    std::cout << "\t#: " << i << "\t" << (devMgr[i])->deviceID.name() << ": " << (devMgr[i])->deviceID.readS() << "\t"  << (devMgr[i])->family.name() << ": " << (devMgr[i])->family.readS() << "\t"  << (devMgr[i])->product.name() << ": " << (devMgr[i])->product.readS() << "\t"  << (devMgr[i])->serial.name() << ": " << (devMgr[i])->serial.readS() << std::endl;
  return 0;
}
