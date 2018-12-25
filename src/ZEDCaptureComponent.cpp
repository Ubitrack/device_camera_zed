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
 * Acquires data zed cameras.
 *
 * @author Ulrich Eck <ulrich.eck@tum.de>
 */

#include "ZEDCaptureComponent.h"
#include <utUtil/OS.h>

// get a logger
static log4cpp::Category &logger(log4cpp::Category::getInstance("Ubitrack.device_camera_zed.ZEDFrameGrabber"));

using namespace Ubitrack;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Drivers;


namespace Ubitrack { namespace Drivers {


/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(sl::Mat &input) {
    // Mapping between sl::MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1:
            cv_type = CV_32FC1;
            break;
        case sl::MAT_TYPE_32F_C2:
            cv_type = CV_32FC2;
            break;
        case sl::MAT_TYPE_32F_C3:
            cv_type = CV_32FC3;
            break;
        case sl::MAT_TYPE_32F_C4:
            cv_type = CV_32FC4;
            break;
        case sl::MAT_TYPE_8U_C1:
            cv_type = CV_8UC1;
            break;
        case sl::MAT_TYPE_8U_C2:
            cv_type = CV_8UC2;
            break;
        case sl::MAT_TYPE_8U_C3:
            cv_type = CV_8UC3;
            break;
        case sl::MAT_TYPE_8U_C4:
            cv_type = CV_8UC4;
            break;
        default:
            break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

void ZEDModule::startModule()
{
    if ( !m_running )
    {
        m_bStop = false;

        // check if oclmanager is active
        Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
        if ((oclManager.isEnabled()) && (oclManager.isActive()) && (!oclManager.isInitialized())) {
            LOG4CPP_INFO(logger, "Waiting for OpenCLManager Initialization callback.");
            oclManager.registerInitCallback(boost::bind(&ZEDModule::startCapturing, this));
        } else {
            startCapturing();
        }
        m_running = true;
    }
}

void ZEDModule::startCapturing() {
    m_Thread.reset(new boost::thread(boost::bind(&ZEDModule::captureThread, this)));
}

void ZEDModule::stopModule()
{
    if ( m_running )
    {
        m_running = false;
        m_bStop = true;
        LOG4CPP_INFO( logger, "Trying to stop ZED module");
        if ( m_Thread )
        {
            m_Thread->join();
            LOG4CPP_INFO( logger, "Trying to stop ZED module, thread joined");
        }
    }
}

boost::shared_ptr< ZEDComponent > ZEDModule::createComponent( const std::string& type,
        const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
        const ComponentKey& key, ModuleClass* pModule )
{

    if ( type == "ZEDVideoStream" )
        return boost::shared_ptr< ZEDComponent >( new ZEDVideoComponent( name, subgraph, key, pModule ) );
//    else if ( type == "ZEDMeasureStream" )
//        return boost::shared_ptr< ZEDComponent >( new ZEDMeasureComponent( name, subgraph, key, pModule ) );

    UBITRACK_THROW( "Class " + type + " not supported by ZED module" );
}

void ZEDModule::captureThread()
{

    LOG4CPP_INFO(logger, "Setting up camera");

    // Set configuration parameters
    sl::InitParameters init_params;
    init_params.camera_resolution = m_cameraResolution;
    init_params.depth_mode = m_depthMode;
    init_params.camera_fps = m_cameraFrameRate;
    init_params.coordinate_units = sl::UNIT_METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE; // same as opencv, origin upper/left corner, right handed
    init_params.sdk_verbose = true;
    if (m_serialNumber != 0) {
        init_params.input.setFromSerialNumber(m_serialNumber);
    }

    {
        // Open the camera
        sl::ERROR_CODE err = m_zedcamera.open(init_params);
        if (err != sl::SUCCESS) {
            LOG4CPP_ERROR(logger, sl::toString(err).c_str());
            m_zedcamera.close();
            return; // Quit if an error occurred
        }

        LOG4CPP_INFO(logger, "ZED Camera setup successful.");
    }

    // initialize components
    ComponentList allComponents = getAllComponents();

    for (auto i = allComponents.begin(); i != allComponents.end(); ++i) {
        (*i)->init(m_zedcamera);
    }


    // Set runtime parameters after opening the camera
    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = m_sensingMode;
    runtime_parameters.enable_depth = true;
    runtime_parameters.enable_point_cloud = true;
    runtime_parameters.measure3D_reference_frame = sl::REFERENCE_FRAME_CAMERA;

    while (!m_bStop) {


#ifdef ENABLE_EVENT_TRACING
        TRACEPOINT_MEASUREMENT_CREATE(getEventDomain(), time, getName().c_str(), "VideoCapture")
#endif
        sl::ERROR_CODE err = m_zedcamera.grab(runtime_parameters);
        if (err != sl::SUCCESS) {
            if (err == sl::ERROR_CODE_NOT_A_NEW_FRAME) {
                Ubitrack::Util::sleep(1);
            } else {
                LOG4CPP_WARN(logger, "Error while grabbing frame: " << err);
            }
            continue;
        }

        sl::timeStamp last_image_timestamp = m_zedcamera.getTimestamp(sl::TIME_REFERENCE_IMAGE);
        Measurement::Timestamp time(last_image_timestamp);

        if (!m_bStop) {

            ComponentList allComponents = getAllComponents();

            for (auto i = allComponents.begin(); i != allComponents.end(); ++i) {
                (*i)->process(time, m_zedcamera);
            }
        }
    }

    if (m_zedcamera.isOpened()) {
        m_zedcamera.close();
    }

}

void ZEDVideoComponent::init(sl::Camera &cam) {

    // Prepare new image size to retrieve half-resolution images
    sl::Resolution image_size = cam.getResolution();
    m_imageWidth = image_size.width;
    m_imageHeight = image_size.height;


    m_imageFormatProperties = Vision::Image::ImageFormatProperties();

    switch (m_componentKey.getVideoSource()) {
        case sl::VIEW_LEFT:
        case sl::VIEW_RIGHT:
        case sl::VIEW_LEFT_UNRECTIFIED:
        case sl::VIEW_RIGHT_UNRECTIFIED:
        case sl::VIEW_SIDE_BY_SIDE:
        case sl::VIEW_DEPTH:
        case sl::VIEW_DEPTH_RIGHT:
        case sl::VIEW_CONFIDENCE:
        case sl::VIEW_NORMALS:
        case sl::VIEW_NORMALS_RIGHT:
            m_mat_type = sl::MAT_TYPE_8U_C4;

            m_imageFormatProperties.depth = CV_8U;
            m_imageFormatProperties.channels = 4;
            m_imageFormatProperties.matType = CV_8UC4;
            m_imageFormatProperties.bitsPerPixel = 32;
            m_imageFormatProperties.origin = 0;
            m_imageFormatProperties.imageFormat = Vision::Image::BGRA;
            break;
        case sl::VIEW_LEFT_GRAY:
        case sl::VIEW_RIGHT_GRAY:
        case sl::VIEW_LEFT_UNRECTIFIED_GRAY:
        case sl::VIEW_RIGHT_UNRECTIFIED_GRAY:
            m_mat_type = sl::MAT_TYPE_8U_C1;

            m_imageFormatProperties.depth = CV_8U;
            m_imageFormatProperties.channels = 1;
            m_imageFormatProperties.matType = CV_8UC1;
            m_imageFormatProperties.bitsPerPixel = 8;
            m_imageFormatProperties.origin = 0;
            m_imageFormatProperties.imageFormat = Vision::Image::LUMINANCE;
            break;
        default:
            LOG4CPP_WARN(logger, "unknown sl::MAT_TYPE ");
    }
    m_image_zed.reset(new sl::Mat(m_imageWidth, m_imageHeight, m_mat_type));

}


void ZEDVideoComponent::process(Measurement::Timestamp ts, sl::Camera &cam) {

    if (m_outputPort.isConnected()) {

        // we're copying twice here - once from driver to app space and then we clone the image ..
        // should be fixed by first allocation the image and referencing the sl::Mat with allocated memory.
        cam.retrieveImage(*m_image_zed, m_componentKey.getVideoSource(), sl::MEM_CPU, m_imageWidth, m_imageHeight);
        boost::shared_ptr <Image> pColorImage;

        // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
        // Only the headers and pointer to the sl::Mat are copied, not the data itself
        cv::Mat image_ref_ocv = slMat2cvMat(*m_image_zed);

        // for now we need to copy the opencv image since ref_ocv refers to a changing sl::Mat buffer
        cv::Mat image = image_ref_ocv.clone();

        pColorImage.reset(new Image(image));
        pColorImage->set_origin(m_imageFormatProperties.origin);
        pColorImage->set_pixelFormat(m_imageFormatProperties.imageFormat);

        if (m_autoGPUUpload) {
            Vision::OpenCLManager &oclManager = Vision::OpenCLManager::singleton();
            if (oclManager.isInitialized()) {
                //force upload to the GPU
                pColorImage->uMat();
            }
        }

        m_outputPort.send(Measurement::ImageMeasurement(ts, pColorImage));
    }

}


void ZEDMeasureComponent::init(sl::Camera &cam) {
//
//    // Prepare new image size to retrieve half-resolution images
//    sl::Resolution image_size = cam.getResolution();
//    m_imageWidth = image_size.width;
//    m_imageHeight = image_size.height;
//
//
//    m_imageFormatProperties = Vision::Image::ImageFormatProperties();
//    sl::MAT_TYPE mat_type = sl::MAT_TYPE_8U_C4;
//
//    switch (m_componentKey.getVideoSource()) {
//        case sl::VIEW_LEFT:
//        case sl::VIEW_RIGHT:
//        case sl::VIEW_LEFT_UNRECTIFIED:
//        case sl::VIEW_RIGHT_UNRECTIFIED:
//        case sl::VIEW_SIDE_BY_SIDE:
//        case sl::VIEW_DEPTH:
//        case sl::VIEW_DEPTH_RIGHT:
//        case sl::VIEW_CONFIDENCE:
//        case sl::VIEW_NORMALS:
//        case sl::VIEW_NORMALS_RIGHT:
//            mat_type = sl::MAT_TYPE_8U_C4;
//
//            m_imageFormatProperties.depth = CV_8U;
//            m_imageFormatProperties.channels = 4;
//            m_imageFormatProperties.matType = CV_8UC4;
//            m_imageFormatProperties.bitsPerPixel = 32;
//            m_imageFormatProperties.origin = 0;
//            m_imageFormatProperties.imageFormat = Vision::Image::RGBA;
//            break;
//        case sl::VIEW_LEFT_GRAY:
//        case sl::VIEW_RIGHT_GRAY:
//        case sl::VIEW_LEFT_UNRECTIFIED_GRAY:
//        case sl::VIEW_RIGHT_UNRECTIFIED_GRAY:
//            mat_type = sl::MAT_TYPE_8U_C1;
//
//            m_imageFormatProperties.depth = CV_8U;
//            m_imageFormatProperties.channels = 1;
//            m_imageFormatProperties.matType = CV_8UC1;
//            m_imageFormatProperties.bitsPerPixel = 8;
//            m_imageFormatProperties.origin = 0;
//            m_imageFormatProperties.imageFormat = Vision::Image::LUMINANCE;
//            break;
//        default:
//        LOG4CPP_WARN(logger, "unknown sl::MAT_TYPE ");
//    }
//    m_image_zed = sl::Mat(m_imageWidth, m_imageHeight, mat_type);
////            sl::Mat point_cloud;


}


void ZEDMeasureComponent::process(Measurement::Timestamp ts, sl::Camera &cam) {
//
//    if (m_outputPort.isConnected()) {
//
//// Retrieve the point cloud
//// To learn how to manipulate and display point clouds, see Depth Sensing sample
////                    m_zedcamera.retrieveMeasure(point_cloud, sl::MEASURE_XYZ, sl::MEM_CPU, new_width, new_height);
////                    LOG4CPP_WARN(logger, "pcl channels: " << point_cloud.getChannels() << " height: " << point_cloud.getHeight()
////                    << " width: " << point_cloud.getWidth() << " bytes: " << point_cloud.getPixelBytes() );
//
//        cam.retrieveImage(getZEDImage(), sl::VIEW_LEFT, sl::MEM_CPU, m_imageWidth, m_imageHeight);
//        boost::shared_ptr <Image> pColorImage;
//
//        // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
//        // Only the headers and pointer to the sl::Mat are copied, not the data itself
//        cv::Mat image_ref_ocv = slMat2cvMat(getZEDImage());
//
//        // for now we need to copy the opencv image since ref_ocv refers to a changing sl::Mat buffer
//        cv::Mat image = image_ref_ocv.clone();
//
//        pColorImage.reset(new Image(image));
//        pColorImage->set_origin(m_imageFormatProperties.origin);
//        pColorImage->set_pixelFormat(m_imageFormatProperties.imageFormat);
//
//        if (m_autoGPUUpload) {
//            Vision::OpenCLManager &oclManager = Vision::OpenCLManager::singleton();
//            if (oclManager.isInitialized()) {
//                //force upload to the GPU
//                pColorImage->uMat();
//            }
//        }
//
//        m_outputPort.send(Measurement::ImageMeasurement(ts, pColorImage));
//    }

}


std::ostream& operator<<( std::ostream& s, const ZEDComponentKey& k )
{
    s << "ZEDComponentKey[ "
      << k.getSensorType() << " "
      << k.getVideoSource() << " "
      << k.getMeasureSource() << " ]";
    return s;
}


// register module at factory
UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
    std::vector< std::string > moduleComponents;
    moduleComponents.emplace_back( "ZEDVideoStream" );
//    moduleComponents.emplace_back( "ZEDPointCloudStream" );
//    moduleComponents.emplace_back( "ZEDCameraIntrinsics" );

    cf->registerModule< Ubitrack::Drivers::ZEDModule > ( moduleComponents );
}


} } // namespace Ubitrack::Drivers