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
int slMatType2cvMatType(sl::MAT_TYPE t) {
    int cv_type = -1;
    switch (t) {
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
    return cv_type;
}

cv::Mat slMat2cvMat(sl::Mat &input) {
    // Mapping between sl::MAT_TYPE and CV_TYPE
    int cv_type = slMatType2cvMatType(input.getDataType());

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

/**
* Conversion function between cv::Mat and sl::Mat
**/
sl::MAT_TYPE cvMatType2slMatType(int t) {
    sl::MAT_TYPE mat_type;
    switch(t) {
        case CV_32FC1:
            mat_type = sl::MAT_TYPE_32F_C1;
            break;
        case CV_32FC2:
            mat_type = sl::MAT_TYPE_32F_C2;
            break;
        case CV_32FC3:
            mat_type = sl::MAT_TYPE_32F_C3;
            break;
        case CV_32FC4:
            mat_type = sl::MAT_TYPE_32F_C4;
            break;
        case CV_8UC1:
            mat_type = sl::MAT_TYPE_8U_C1;
            break;
        case CV_8UC2:
            mat_type = sl::MAT_TYPE_8U_C2;
            break;
        case CV_8UC3:
            mat_type = sl::MAT_TYPE_8U_C3;
            break;
        case CV_8UC4:
            mat_type = sl::MAT_TYPE_8U_C4;
            break;
        default:
            break;
    }
    return mat_type;
}

sl::Mat cvMat2slMat(cv::Mat &input) {
    // Mapping between sl::MAT_TYPE and CV_TYPE
    sl::MAT_TYPE mat_type = cvMatType2slMatType(input.type());
    // cv::Mat and sl::Mat will share a single memory structure
    return sl::Mat(input.cols, input.rows, mat_type, input.data, input.elemSize()*input.cols, sl::MEM_CPU);
}

Math::Quaternion slRotation2utQuaternion(sl::Rotation &input) {
    // Convert rotation sl::Rotation into Math::Quaternion
    sl::Orientation sl_orn(input);
    Math::Quaternion ut_quat((double)sl_orn[0], (double)sl_orn[1], (double)sl_orn[2], (double)sl_orn[3]);
    return ut_quat;
}

Math::CameraIntrinsics<double> slCameraParameters2utCameraIntrinsics(sl::CameraParameters &input) {
    // Convert camera calibration sl::CameraParameters into Math::CameraIntrinsics

    LOG4CPP_INFO(logger, "ZED Instrinsics hfov: " << input.h_fov << " vfov: " << input.v_fov
            << " fx: " << input.fx << " fy: " << input.fy
            << " cx: " << input.cx << " cy: " << input.cy
            << " width: " << input.image_size.width << " height: " << input.image_size.height);

    Math::Matrix< double, 3, 3 > intrinsicMatrix = Math::Matrix3x3d::identity();
    intrinsicMatrix(0, 0) = input.fx;
    intrinsicMatrix(1, 1) = input.fy;
    intrinsicMatrix(0, 2) = -input.cx;
    intrinsicMatrix(1, 2) = -input.cy;
    intrinsicMatrix(2, 2) = -1.0;

    // [ k1, k2, p1, p2, k3 ]
    Math::Vector< double, 3 > radial(input.disto[0], input.disto[1], input.disto[4]);
    Math::Vector< double, 2 > tangential(input.disto[2], input.disto[3]);
    std::size_t width = input.image_size.width;
    std::size_t height = input.image_size.height;


    Math::CameraIntrinsics<double> ut_intr(intrinsicMatrix, radial, tangential, width, height);
    return ut_intr;
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
    else if ( type == "ZEDCameraCalibration" )
        return boost::shared_ptr< ZEDComponent >( new ZEDCameraCalibrationComponent( name, subgraph, key, pModule ) );
    else if ( type == "ZEDPointCloud" )
        return boost::shared_ptr< ZEDComponent >( new ZEDPointCloudComponent( name, subgraph, key, pModule ) );
    else if ( type == "ZEDDepthMap" )
        return boost::shared_ptr< ZEDComponent >( new ZEDDepthMapComponent( name, subgraph, key, pModule ) );
    else if ( type == "ZEDRecorder" )
        return boost::shared_ptr< ZEDComponent >( new ZEDRecorderComponent( name, subgraph, key, pModule ) );

    UBITRACK_THROW( "Class " + type + " not supported by ZED module" );
}

void ZEDModule::captureThread()
{

    LOG4CPP_INFO(logger, "Setting up camera");

    // Set configuration parameters
    sl::InitParameters init_params;

#ifdef ENABLE_EVENT_TRACING
    int eventdomain = -1;
#endif

    // pull configuration
    ComponentList allComponents = getAllComponents();

    for (auto i = allComponents.begin(); i != allComponents.end(); ++i) {
        (*i)->configure(init_params);
#ifdef ENABLE_EVENT_TRACING
        if (eventdomain == -1) {
            eventdomain = (*i)->getEventDomain();
        } else if (eventdomain != (*i)->getEventDomain()) {
            LOG4CPP_WARN(logger, "Multiple Components of the same module are in different event domains !!!");
        }
#endif
    }

    // if device is not configured for playback, configure sensor as follows
    if (init_params.svo_input_filename.empty()) {
        init_params.camera_resolution = m_cameraResolution;
        init_params.depth_mode = m_depthMode;
        init_params.camera_fps = m_cameraFrameRate;

        if (m_serialNumber != 0) {
            init_params.input.setFromSerialNumber(m_serialNumber);
        }
    }

    init_params.coordinate_units = sl::UNIT_MILLIMETER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    init_params.sdk_verbose = true;

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
        TRACEPOINT_MEASUREMENT_CREATE(eventdomain, time, "zed_camera", "VideoCapture")
#endif
        sl::ERROR_CODE err = m_zedcamera.grab(runtime_parameters);
        if (err != sl::SUCCESS) {
            if (err == sl::ERROR_CODE_NOT_A_NEW_FRAME) {
                Ubitrack::Util::sleep(5);
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

    // teardown components
    for (auto i = allComponents.begin(); i != allComponents.end(); ++i) {
        (*i)->teardown(m_zedcamera);
    }

    if (m_zedcamera.isOpened()) {
        m_zedcamera.close();
    }

}

void ZEDVideoComponent::init(sl::Camera &cam) {

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

}


void ZEDVideoComponent::process(Measurement::Timestamp ts, sl::Camera &cam) {

    if (m_outputPort.isConnected()) {


        cv::Mat image_ocv(m_imageHeight, m_imageWidth, m_imageFormatProperties.matType);

        sl::Mat image_zed = cvMat2slMat(image_ocv);
        sl::ERROR_CODE err = cam.retrieveImage(image_zed, m_componentKey.getVideoSource(), sl::MEM_CPU, m_imageWidth, m_imageHeight);
        if (err != sl::SUCCESS) {
            LOG4CPP_ERROR(logger, "Error while grabbing frame: " << sl::toString(err));
            return;
        }

        boost::shared_ptr< Vision::Image > pColorImage(new Vision::Image(image_ocv));
        pColorImage->set_pixelFormat(m_imageFormatProperties.imageFormat);
        pColorImage->set_origin(m_imageFormatProperties.origin);

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



void ZEDCameraCalibrationComponent::init(sl::Camera &cam) {

    sl::CameraInformation zed_camera_info = cam.getCameraInformation();
    sl::CalibrationParameters zed_camera_calibration = zed_camera_info.calibration_parameters;

    // convert left-to-right transform to ubitrack
    sl::Rotation zed_lr_rot;
    zed_lr_rot.setRotationVector(zed_camera_calibration.R);

    Math::Quaternion lr_orn = slRotation2utQuaternion(zed_lr_rot);
    Math::Vector3d lr_trn(
            (double)zed_camera_calibration.T[0],
            (double)zed_camera_calibration.T[1],
            (double)zed_camera_calibration.T[2]
            );

    m_leftToRightTransform = Math::Pose(lr_orn, lr_trn);

    m_leftCameraIntrinsics = slCameraParameters2utCameraIntrinsics(zed_camera_calibration.left_cam);
    m_rightCameraIntrinsics = slCameraParameters2utCameraIntrinsics(zed_camera_calibration.right_cam);

}


void ZEDPointCloudComponent::init(sl::Camera &cam) {
//
    sl::Resolution image_size = cam.getResolution();
    m_imageWidth = image_size.width;
    m_imageHeight = image_size.height;
    m_numberPoints = image_size.area();

    switch (m_componentKey.getMeasureSource()) {
        case sl::MEASURE_DISPARITY: /**< Disparity map. Each pixel contains 1 float. sl::MAT_TYPE_32F_C1.*/
        case sl::MEASURE_DISPARITY_RIGHT: /**< Disparity map for right sensor. Each pixel contains 1 float. sl::MAT_TYPE_32F_C1.*/
        case sl::MEASURE_DEPTH: /**< Depth map. Each pixel contains 1 float. sl::MAT_TYPE_32F_C1.*/
        case sl::MEASURE_DEPTH_RIGHT: /**< Depth map for right sensor. Each pixel contains 1 float. sl::MAT_TYPE_32F_C1.*/
        case sl::MEASURE_CONFIDENCE: /**< Certainty/confidence of the depth map. Each pixel contains 1 float. sl::MAT_TYPE_32F_C1.*/
            m_mat_type = sl::MAT_TYPE_32F_C1;

            break;

        case sl::MEASURE_XYZ: /**< Point cloud. Each pixel contains 4 float (X, Y, Z, not used). sl::MAT_TYPE_32F_C4.*/
        case sl::MEASURE_XYZRGBA: /**< Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the RGBA color.  sl::MAT_TYPE_32F_C4.*/
        case sl::MEASURE_XYZBGRA: /**< Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the BGRA color.  sl::MAT_TYPE_32F_C4.*/
        case sl::MEASURE_XYZARGB: /**< Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the ARGB color.  sl::MAT_TYPE_32F_C4.*/
        case sl::MEASURE_XYZABGR: /**< Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the ABGR color.  sl::MAT_TYPE_32F_C4.*/
        case sl::MEASURE_NORMALS: /**< Normals vector. Each pixel contains 4 float (X, Y, Z, 0).  sl::MAT_TYPE_32F_C4.*/
        case sl::MEASURE_XYZ_RIGHT: /**< Point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, not used). sl::MAT_TYPE_32F_C4.*/
        case sl::MEASURE_XYZRGBA_RIGHT: /**< Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the RGBA color. sl::MAT_TYPE_32F_C4.*/
        case sl::MEASURE_XYZBGRA_RIGHT: /**< Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the BGRA color. sl::MAT_TYPE_32F_C4.*/
        case sl::MEASURE_XYZARGB_RIGHT: /**< Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the ARGB color. sl::MAT_TYPE_32F_C4.*/
        case sl::MEASURE_XYZABGR_RIGHT: /**< Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the ABGR color. sl::MAT_TYPE_32F_C4.*/
        case sl::MEASURE_NORMALS_RIGHT: /**< Normals vector for right view. Each pixel contains 4 float (X, Y, Z, 0).  sl::MAT_TYPE_32F_C4.*/
            m_mat_type = sl::MAT_TYPE_32F_C4;

            break;

        default:
        LOG4CPP_WARN(logger, "unknown sl::MAT_TYPE ");
    }

    LOG4CPP_INFO(logger, "Initialized Pointcloud Component with " << m_numberPoints << " points.");
    m_pointcloud_zed.reset(new sl::Mat(m_imageWidth, m_imageHeight, m_mat_type));

}


void ZEDPointCloudComponent::process(Measurement::Timestamp ts, sl::Camera &cam) {


    if (m_outputPort.isConnected()) {

        // we're copying twice here - once from driver to app space and then we clone the image ..
        // should be fixed by first allocation the image and referencing the sl::Mat with allocated memory.
        sl::ERROR_CODE err = cam.retrieveMeasure(*m_pointcloud_zed, m_componentKey.getMeasureSource(), sl::MEM_CPU, m_imageWidth, m_imageHeight);
        if (err != sl::SUCCESS) {
            LOG4CPP_WARN(logger, "Error while grabbing frame: " << sl::toString(err));
        }

        Math::Vector3d init_pos(0, 0, 0);
        boost::shared_ptr < std::vector<Math::Vector3d> > pPointCloud = boost::make_shared< std::vector<Math::Vector3d> >(m_numberPoints, init_pos);


        auto *p_data_cloud = m_pointcloud_zed->getPtr<float>();
        int index = 0;

        for (size_t i = 0; i < m_numberPoints; i++) {
            float X = p_data_cloud[index];
            Math::Vector3d& p = pPointCloud->at(i);

            if (!isValidMeasure(X)) // Checking if it's a valid point
                p[0] = p[1] = p[2] = 0.;
            else {
                p[0] = X;
                p[1] = p_data_cloud[index + 1];
                p[2] = p_data_cloud[index + 2];
                // ignore: p_data_cloud[index + 3] - contains rgba 8UC3 as 32bit-float
            }
            index += 4;
        }

        m_outputPort.send(Measurement::PositionList(ts, pPointCloud));
    }
}



void ZEDDepthMapComponent::init(sl::Camera &cam) {

    sl::Resolution image_size = cam.getResolution();
    m_imageWidth = image_size.width;
    m_imageHeight = image_size.height;


    m_imageFormatProperties = Vision::Image::ImageFormatProperties();

    switch (m_componentKey.getMeasureSource()) {
        case sl::MEASURE_DISPARITY: /**< Disparity map. Each pixel contains 1 float. sl::MAT_TYPE_32F_C1.*/
        case sl::MEASURE_DISPARITY_RIGHT: /**< Disparity map for right sensor. Each pixel contains 1 float. sl::MAT_TYPE_32F_C1.*/
        case sl::MEASURE_DEPTH: /**< Depth map. Each pixel contains 1 float. sl::MAT_TYPE_32F_C1.*/
        case sl::MEASURE_DEPTH_RIGHT: /**< Depth map for right sensor. Each pixel contains 1 float. sl::MAT_TYPE_32F_C1.*/
        case sl::MEASURE_CONFIDENCE: /**< Certainty/confidence of the depth map. Each pixel contains 1 float. sl::MAT_TYPE_32F_C1.*/
            m_imageFormatProperties.depth = CV_32F;
            m_imageFormatProperties.channels = 1;
            m_imageFormatProperties.matType = CV_32FC1;
            m_imageFormatProperties.bitsPerPixel = 32;
            m_imageFormatProperties.origin = 0;
            m_imageFormatProperties.imageFormat = Vision::Image::DEPTH;
            break;
        default:
            LOG4CPP_WARN(logger, "unknown MeasureSource.. ");
    }

}


void ZEDDepthMapComponent::process(Measurement::Timestamp ts, sl::Camera &cam) {

    if (m_outputPort.isConnected()) {


        cv::Mat image_ocv(m_imageHeight, m_imageWidth, m_imageFormatProperties.matType);

        sl::Mat image_zed = cvMat2slMat(image_ocv);
        sl::ERROR_CODE err = cam.retrieveMeasure(image_zed, m_componentKey.getMeasureSource(), sl::MEM_CPU, m_imageWidth, m_imageHeight);
        if (err != sl::SUCCESS) {
            LOG4CPP_ERROR(logger, "Error while grabbing frame: " << sl::toString(err));
            return;
        }

        boost::shared_ptr< Vision::Image > pDepthImage(new Vision::Image(image_ocv));
        pDepthImage->set_pixelFormat(m_imageFormatProperties.imageFormat);
        pDepthImage->set_origin(m_imageFormatProperties.origin);

        if (m_autoGPUUpload) {
            Vision::OpenCLManager &oclManager = Vision::OpenCLManager::singleton();
            if (oclManager.isInitialized()) {
                //force upload to the GPU
                pDepthImage->uMat();
            }
        }

        m_outputPort.send(Measurement::ImageMeasurement(ts, pDepthImage));
    }

}



void ZEDRecorderComponent::configure(sl::InitParameters& params) {
    if (m_playback_enabled) {
        if ((!m_svo_player_filename.empty()) && ( boost::filesystem::is_regular_file( m_svo_player_filename ) ) ) {
            LOG4CPP_WARN(logger, "ZED Camera is in Playback Mode: "<< m_svo_player_filename.string());
            params.svo_input_filename.set(m_svo_player_filename.string().c_str());
            params.svo_real_time_mode = true;
        } else {
            LOG4CPP_WARN(logger, "ZED Player - File not found: " <<  m_svo_player_filename.string())
        }
    }
}


void ZEDRecorderComponent::init(sl::Camera &cam) {

    if (m_recording_enabled) {
        LOG4CPP_INFO(logger, "Initializing Recorder Component.");
        sl::String path_output(m_svo_filename.string().c_str());

        LOG4CPP_DEBUG(logger, "Activate Recroding to SVO Filename:" << m_svo_filename.string());
        sl::ERROR_CODE err = cam.enableRecording(path_output, m_svo_compression_mode);

        if (err != sl::SUCCESS) {
            LOG4CPP_ERROR(logger, "Recording initialization error. " << toString(err));
            if (err == sl::ERROR_CODE_SVO_RECORDING_ERROR)
                LOG4CPP_ERROR(logger, " Note : This error mostly comes from a wrong path or missing writing permissions. Also did you specify a filename, not a directory?");
            return;
        }

        LOG4CPP_DEBUG(logger, "Open Timestamp Filename:" << m_timestamp_filename.string());
        m_timestamp_filebuffer.open ( m_timestamp_filename.string().c_str(), std::ios::out );
        if( !m_timestamp_filebuffer.is_open()  ) {
            LOG4CPP_ERROR(logger,  "Error opening timestamp file" );
            return;
        }

        m_recording_active = true;
        m_frame_counter = 0;

    }
    LOG4CPP_DEBUG(logger, "Initialized Recorder Component.");
}


void ZEDRecorderComponent::process(Measurement::Timestamp ts, sl::Camera &cam) {
    if (m_recording_active) {

        // record a frame using zed sdk
        cam.record();

        // write the corresponding timestamp to a file
        std::ostream os( &m_timestamp_filebuffer );
        os << ts << " " << m_frame_counter << std::endl;

        // increase the frame counter
        m_frame_counter++;
    }
}

void ZEDRecorderComponent::teardown(sl::Camera &cam) {
    if (m_recording_active) {
        cam.disableRecording();
        m_recording_active = false;
    }
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
    moduleComponents.emplace_back( "ZEDCameraCalibration" );
    moduleComponents.emplace_back( "ZEDPointCloud" );
    moduleComponents.emplace_back( "ZEDDepthMap" );
    moduleComponents.emplace_back( "ZEDRecorder" );

    cf->registerModule< Ubitrack::Drivers::ZEDModule > ( moduleComponents );
}


} } // namespace Ubitrack::Drivers