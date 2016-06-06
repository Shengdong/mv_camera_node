#include "mvCamInterface.h"
#include <iostream>
#include "sensor_msgs/Image.h"


mvCamInterface::mvCamInterface(mvIMPACT::acquire::Device* _dev)
 : m_dev(_dev)
{
}

bool
mvCamInterface::init(ros::NodeHandle& nh)
{
    try
    {
        m_dev->open();
    }
    catch(ImpactAcquireException& e)
    {
        std::cout<< "An error occurs. Now Exit" <<std::endl;
        return false;
    }      
    std::cout << m_dev->family.read()<<"(" << m_dev->serial.read() << ")" <<std::endl;

    fi.reset(new mvIMPACT::acquire::FunctionInterface(m_dev));
    cs.reset(new mvIMPACT::acquire::CameraSettingsBlueFOX(m_dev));
    ss.reset(new mvIMPACT::acquire::SystemSettings(m_dev));
    io.reset(new mvIMPACT::acquire::IOSubSystemBlueFOX(m_dev));
    cs->imageRequestTimeout_ms.write( 10000 ); 
    cs->expose_us.write(300);
    setFrameRate(50.0);
    image_transport::ImageTransport it(nh);
    m_imagePub = it.advertise("image_rect", 1);
    return true;
}

void
mvCamInterface::destroy(void)
{
    if (m_dev)
    {
        if(m_dev->isOpen())
        {
            m_dev->close();
        }
    }
}

bool
mvCamInterface::setFrameRate(float frameRate)
{
    cs->triggerSource.write(mvIMPACT::acquire::ctsRTCtrl);
    cs->triggerMode.write(mvIMPACT::acquire::ctmOnRisingEdge);

    if (io->RTCtrProgramCount() == 0)
    {
        fprintf(stderr, "# ERROR: setFrameRate() only works if a HRTC controller is available.\n");
        return false;
    }

    mvIMPACT::acquire::RTCtrProgram* program = io->getRTCtrProgram(0);
    if (!program)
    {
        // this should only happen if the system is short of memory
        return false;
    }

    program->mode.write(mvIMPACT::acquire::rtctrlModeStop);
    program->setProgramSize(5);

    int i = 0;
    mvIMPACT::acquire::RTCtrProgramStep* step = program->programStep(i++);
    step->opCode.write(mvIMPACT::acquire::rtctrlProgWaitClocks);
    step->clocks_us.write(static_cast<int>(1000000.0 / frameRate) - triggerPulseWidth());

    step = program->programStep(i++);
    step->opCode.write(mvIMPACT::acquire::rtctrlProgTriggerSet);
    step->frameID.write(1);

    step = program->programStep(i++);
    step->opCode.write(mvIMPACT::acquire::rtctrlProgWaitClocks);
    step->clocks_us.write(triggerPulseWidth());

    step = program->programStep(i++);
    step->opCode.write(mvIMPACT::acquire::rtctrlProgTriggerReset);

    step = program->programStep(i++);
    step->opCode.write(mvIMPACT::acquire::rtctrlProgJumpLoc);

    program->mode.write(mvIMPACT::acquire::rtctrlModeRun);

    return true;
}

int
mvCamInterface::triggerPulseWidth(void) const
{
    int rowLength = cs->aoiWidth.read();

    int pixelClock_KHz = cs->pixelClock_KHz.read();
    double pixelClock_MHz = static_cast<double>(pixelClock_KHz) / 1000.0;

    double rowTime = static_cast<double>(rowLength) / pixelClock_MHz;

    // signal pulse width should be ten times the row time
    return static_cast<int>(round(rowTime * 10.0));
}

void
mvCamInterface::imageHandler(void)
{
    int maxRequest = 1;
    ss->requestCount.write(maxRequest);

    for (int i=0; i<maxRequest;++i)
    {
        fi->imageRequestSingle();
    }

    while(ros::ok())
    {
        int requestNr = fi->imageRequestWaitFor(-1);
        if (fi->isRequestNrValid(requestNr))
        {
            const mvIMPACT::acquire::Request* request = fi->getRequest(requestNr);
            if (request->isOK())
            {
                cv::Mat temp(cv::Size(request->imageWidth.read()*request->imagePixelPitch.read(), request->imageHeight.read()), CV_8UC1, request->imageData.read());
                temp.copyTo(m_image);       
                m_image_ptr = cv_bridge::CvImage(std_msgs::Header(), "mono8", m_image).toImageMsg();
                m_image_ptr->header.stamp = ros::Time::now();
                m_imagePub.publish(m_image_ptr);              
            }
            fi->imageRequestUnlock(requestNr);
            fi->imageRequestSingle();
        }
    }
}


