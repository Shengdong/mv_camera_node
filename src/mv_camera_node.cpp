#include "mvCamInterface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mv_camera");
     
    ros::NodeHandle nh;
    
    mvIMPACT::acquire::DeviceManager devMgr;
    mvIMPACT::acquire::Device* pDev;

    if (devMgr.deviceCount())
    {
        pDev = devMgr[0];
    }
    else
    {
       ROS_WARN("No device Connected");
       return 1;
    }

    mvCamInterface m_cap(pDev);
    if (!m_cap.init(nh))
    {
       ROS_WARN("Initialise device Failed");
       return 1;
    }
    m_cap.imageHandler();
}
