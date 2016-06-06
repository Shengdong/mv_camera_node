#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <boost/shared_ptr.hpp>
#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


class mvCamInterface
{
public:
    explicit mvCamInterface(mvIMPACT::acquire::Device* _dev);

    bool init(ros::NodeHandle& nh);
    void destroy(void);
    void imageHandler(void);

private:
    bool setFrameRate(float frameRate);
    int  triggerPulseWidth(void) const;
    
    mvIMPACT::acquire::Device* m_dev;
    boost::shared_ptr<mvIMPACT::acquire::FunctionInterface> fi;
    boost::shared_ptr<mvIMPACT::acquire::CameraSettingsBlueFOX> cs;
    boost::shared_ptr<mvIMPACT::acquire::SystemSettings> ss;
    boost::shared_ptr<mvIMPACT::acquire::IOSubSystemBlueFOX> io;
    
    image_transport::Publisher m_imagePub;
    sensor_msgs::ImagePtr m_image_ptr;
    cv::Mat m_image;
};
