#include "mvCamInterface.h"

int main(int argc, char **argv)
{
    mvIMPACT::acquire::DeviceManager devMgr;
    mvIMPACT::acquire::Device* pDev;

    if (devMgr.deviceCount())
    {
        pDev = devMgr[0];
    }
    else
    {
       std::cout << "No device Connected" << std::endl;
       return 1;
    }

    mvCamInterface m_cap(pDev);
    if (!m_cap.init())
    {
       std::cout << "Initialise device Failed" << std::endl;
       return 1;
    }
    m_cap.imageHandler();
}
