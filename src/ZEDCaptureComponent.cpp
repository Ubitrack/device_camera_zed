/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Acquires stereo images from cameras using opencv.
 *
 * @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
 */

#include <string>
#include <list>
#include <iostream>
#include <algorithm>
#include <log4cpp/Category.hh>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tuple/tuple.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/OS.h>
#include <utUtil/TracingProvider.h>
#include <utVision/Image.h>
#include <utVision/Undistortion.h>

#include <utVision/OpenCLManager.h>

 // ZED includes
#include <sl_zed/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.device_camera_zed.ZEDFrameGrabber" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;


namespace Ubitrack { namespace Drivers {



/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between sl::MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}


/**
 * @ingroup vision_components
 * Pushes images from one camera using libdc1394.
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * \c ColorOutput push port of type Ubitrack::Vision::Measurement::ImageMeasurement.
 * \c GreyOutput  push port of type Ubitrack::Vision::Measurement::ImageMeasurement.
 */
class ZEDFrameGrabber
    : public Dataflow::Component
{
public:

    /** constructor */
    ZEDFrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

    /** destructor, waits until thread stops */
    ~ZEDFrameGrabber();

    /** Component start method. starts the thread */
    virtual void start();

    /** Component stop method, stops thread */
    virtual void stop();

    /** handler method for incoming pull requests */

    // Measurement::Matrix3x3 getIntrinsic( Measurement::Timestamp t )
    // {
    //     if (m_undistorter) {
    //         return Measurement::Matrix3x3( t, m_undistorter->getMatrix() );
    //     } else {
    //         UBITRACK_THROW( "No undistortion configured for ZEDFrameGrabber" );
    //     }
    // }

    // Measurement::CameraIntrinsics getCameraModel( Measurement::Timestamp t )
    // {
    //     if (m_undistorter) {
    //         return Measurement::CameraIntrinsics( t, m_undistorter->getIntrinsics() );
    //     } else {
    //         UBITRACK_THROW( "No undistortion configured for ZEDFrameGrabber" );
    //     }
    // }

protected:
    // thread main loop
    void ThreadProc();

    // helper functions
    int  camera_setup();
    void camera_cleanup();

    // the thread
    boost::scoped_ptr< boost::thread > m_Thread;

    // stop the thread?
    volatile bool m_bStop;

    // the ports
    Dataflow::PushSupplier< Measurement::ImageMeasurement > m_colorLeftPort;
    Dataflow::PushSupplier< Measurement::ImageMeasurement > m_greyLeftPort;
    // Dataflow::PushSupplier< Measurement::ImageMeasurement > m_colorRightPort;
    // Dataflow::PushSupplier< Measurement::ImageMeasurement > m_greyRightPort;
    Dataflow::PushSupplier< Measurement::ImageMeasurement > m_depthLeftPort;
    Dataflow::PushSupplier< Measurement::PositionList > m_pointcloudPort;
    // Dataflow::PullSupplier< Measurement::Matrix3x3 > m_intrinsicsPort;
    // Dataflow::PullSupplier< Measurement::CameraIntrinsics > m_cameraModelPort;

    sl::Camera m_zedcamera;

    unsigned int m_width, m_height;
    bool m_autoGPUUpload;

};


void ZEDFrameGrabber::stop()
{
    LOG4CPP_TRACE( logger, "Stopping thread..." );

    if ( m_running )
    {
        LOG4CPP_TRACE( logger, "Thread was running" );

        if ( m_Thread )
        {
            m_bStop = true;
            m_Thread->join();
        }
        m_running = false;
    }
}


void ZEDFrameGrabber::start()
{
    LOG4CPP_TRACE( logger, "Starting thread..." );

    if ( !m_running )
    {
        m_bStop = false;
        m_Thread.reset( new boost::thread( boost::bind ( &ZEDFrameGrabber::ThreadProc, this ) ) );
        m_running = true;
    }
}


ZEDFrameGrabber::ZEDFrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
    : Dataflow::Component( sName )
    , m_bStop( true )
    , m_colorLeftPort( "ColorLeftOutput", *this )
    , m_greyLeftPort( "GrayLeftOutput", *this )
    // , m_colorRightPort( "ColorRightOutput", *this )
    // , m_greyRightPort( "RightOutput", *this )
    , m_depthLeftPort( "DepthLeftOutput", *this )
    , m_pointcloudPort( "PointCloud", *this )
    // , m_intrinsicsPort( "Intrinsics", *this, boost::bind( &ZEDFrameGrabber::getIntrinsic, this, _1 ) )
    // , m_cameraModelPort( "CameraModel", *this, boost::bind( &ZEDFrameGrabber::getCameraModel, this, _1 ) )
    , m_autoGPUUpload(false)

{
    
    Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
    if (oclManager.isEnabled()) {
        if (subgraph->m_DataflowAttributes.hasAttribute("uploadImageOnGPU")){
            m_autoGPUUpload = subgraph->m_DataflowAttributes.getAttributeString("uploadImageOnGPU") == "true";
            LOG4CPP_INFO(logger, "Upload to GPU enabled? " << m_autoGPUUpload);
        }
        if (m_autoGPUUpload){
            oclManager.activate();
            LOG4CPP_INFO(logger, "Require OpenCLManager");
        }
    }

    stop();
}


ZEDFrameGrabber::~ZEDFrameGrabber()
{
    stop();
}


void ZEDFrameGrabber::camera_cleanup()
{
    if (m_zedcamera.isOpened()) {
        m_zedcamera.close();
    }
    LOG4CPP_TRACE( logger, "cleanup finished." );
}


int ZEDFrameGrabber::camera_setup()
{
    LOG4CPP_INFO( logger, "Setting up camera" );

    // Set configuration parameters
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION_HD2K;
    init_params.depth_mode = sl::DEPTH_MODE_ULTRA;
    init_params.coordinate_units = sl::UNIT_METER;

    // Open the camera
    sl::ERROR_CODE err = m_zedcamera.open(init_params);
    if (err != sl::SUCCESS) {
        LOG4CPP_WARN(logger, sl::toString(err).c_str());
        m_zedcamera.close();
        return 0; // Quit if an error occurred
    }

    LOG4CPP_INFO( logger, "ZED Camera setup successful." );
    return 1;
}




void ZEDFrameGrabber::ThreadProc()
{
    LOG4CPP_INFO(logger, "framegrabber thread starting...");
    m_running = true;

    //  setup camera
    if (!camera_setup()) return;



    // Set runtime parameters after opening the camera
    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD;

    // Prepare new image size to retrieve half-resolution images
    sl::Resolution image_size = m_zedcamera.getResolution();
    int new_width = image_size.width;
    int new_height = image_size.height;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    sl::Mat image_zed_left(new_width, new_height, sl::MAT_TYPE_8U_C4);
    cv::Mat image_ocv_left = slMat2cvMat(image_zed_left);
    sl::Mat depth_image_zed_left(new_width, new_height, sl::MAT_TYPE_8U_C4);
    cv::Mat depth_image_ocv_left = slMat2cvMat(depth_image_zed_left);
    sl::Mat point_cloud;

    while ( !m_bStop )
    {


#ifdef ENABLE_EVENT_TRACING
            TRACEPOINT_MEASUREMENT_CREATE(getEventDomain(), time, getName().c_str(), "VideoCapture")
#endif

        if (m_zedcamera.grab(runtime_parameters) != sl::SUCCESS) {
            LOG4CPP_WARN(logger, "Can not grab frame.");
            continue;
        }
        if ( !m_bStop ) {
            /* store the last frame's timestamp */
            Measurement::Timestamp time = Measurement::now();

            if (m_colorLeftPort.isConnected()) {
                m_zedcamera.retrieveImage(image_zed_left, sl::VIEW_LEFT, sl::MEM_CPU, new_width, new_height);
                boost::shared_ptr< Image > pColorImage;
                boost::shared_ptr< Image > pGreyImage;
                pColorImage.reset( new Image( image_ocv_left ) );
                pColorImage->set_origin(0);
                pColorImage->set_pixelFormat(Vision::Image::BGRA);

                if (m_autoGPUUpload){
                    Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
                    if (oclManager.isInitialized()) {
                        //force upload to the GPU
                        pColorImage->uMat();
                    }
                }

                m_colorLeftPort.send( Measurement::ImageMeasurement( time, pColorImage ) );

                if (m_greyLeftPort.isConnected()) {
                    pGreyImage = pColorImage->CvtColor( CV_RGB2GRAY, 1 );
                    if (m_autoGPUUpload){
                        Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
                        if (oclManager.isInitialized()) {
                            //force upload to the GPU
                            pGreyImage->uMat();
                        }
                    }
                    m_greyLeftPort.send( Measurement::ImageMeasurement( time, pGreyImage ) );
                }
            }


            if (m_depthLeftPort.isConnected()) {
                m_zedcamera.retrieveImage(depth_image_zed_left, sl::VIEW_DEPTH, sl::MEM_CPU, new_width, new_height);
                boost::shared_ptr< Image > pColorImage;
                pColorImage.reset( new Image( depth_image_ocv_left ) );
                pColorImage->set_origin(0);
                pColorImage->set_pixelFormat(Vision::Image::BGRA);

                if (m_autoGPUUpload){
                    Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
                    if (oclManager.isInitialized()) {
                        //force upload to the GPU
                        pColorImage->uMat();
                    }
                }

                m_depthLeftPort.send( Measurement::ImageMeasurement( time, pColorImage ) );
            }


            // Retrieve the point cloud
            // To learn how to manipulate and display point clouds, see Depth Sensing sample
            // zed.retrieveMeasure(point_cloud, MEASURE_XYZ, MEM_CPU, new_width, new_height);

        }

    }
    LOG4CPP_INFO(logger, "framegrabber thread finished...");
    camera_cleanup();
}

} } // namespace Ubitrack::Driver


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
    cf->registerComponent< Ubitrack::Drivers::ZEDFrameGrabber > ( "ZEDFrameGrabber" );
}


