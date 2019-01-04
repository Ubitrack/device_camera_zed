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
 * @ingroup driver_components
 * @file
 * ZED Stereolabs driver
 * This file contains the driver component to
 * synchronously capture camera images using ZED SDK.
 *
 * The received data is sent via a push interface.
 *
 * @author Ulrich Eck <ulrick.eck@tum.de>
 */


#ifndef UBITRACK_DEVICE_CAMERA_ZED_ZEDCAPTURECOMPONENT_H
#define UBITRACK_DEVICE_CAMERA_ZED_ZEDCAPTURECOMPONENT_H


#include <string>
#include <cstdlib>

#include <iostream>
#include <memory>
#include <map>
#include <boost/array.hpp>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/Component.h>
#include <utDataflow/Module.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utVision/OpenCLManager.h>
#include <utUtil/TracingProvider.h>

#include <utVision/Image.h>

// ZED includes
#include <sl_zed/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>

namespace {


    enum ZEDSensorType {
        ZED_SENSOR_CONFIG = 0,
        ZED_SENSOR_VIDEO,
        ZED_SENSOR_DEPTH,
        ZED_SENSOR_TRACKING,
        ZED_SENSOR_MAPPING
    };


    class ZEDSensorTypeMap : public std::map<std::string, ZEDSensorType > {
    public:
        ZEDSensorTypeMap() {
            (*this)["CONFIG"] = ZED_SENSOR_CONFIG;
            (*this)["VIDEO"] = ZED_SENSOR_VIDEO;
            (*this)["DEPTH"] = ZED_SENSOR_DEPTH;
            (*this)["TRACKING"] = ZED_SENSOR_TRACKING;
            (*this)["MAPPING"] = ZED_SENSOR_MAPPING;
        }
    };
    static ZEDSensorTypeMap zedSensorTypeMap;

    class ZEDStreamResolutionMap : public std::map<std::string, sl::RESOLUTION > {
    public:
        ZEDStreamResolutionMap() {
            (*this)["RESOLUTION_HD2K"] = sl::RESOLUTION_HD2K;
            (*this)["RESOLUTION_HD1080"] = sl::RESOLUTION_HD1080;
            (*this)["RESOLUTION_HD720"] = sl::RESOLUTION_HD720;
            (*this)["RESOLUTION_VGA"] = sl::RESOLUTION_VGA;
        }
    };
    static ZEDStreamResolutionMap zedStreamResolutionMap;

    class ZEDVideoSourceMap : public std::map<std::string, sl::VIEW > {
    public:
        ZEDVideoSourceMap() {
            (*this)["VIEW_LEFT"] = sl::VIEW_LEFT;
            (*this)["VIEW_RIGHT"] = sl::VIEW_RIGHT;
            (*this)["VIEW_LEFT_GRAY"] = sl::VIEW_LEFT_GRAY;
            (*this)["VIEW_RIGHT_GRAY"] = sl::VIEW_RIGHT_GRAY;
            (*this)["VIEW_LEFT_UNRECTIFIED"] = sl::VIEW_LEFT_UNRECTIFIED;
            (*this)["VIEW_RIGHT_UNRECTIFIED"] = sl::VIEW_RIGHT_UNRECTIFIED;
            (*this)["VIEW_LEFT_UNRECTIFIED_GRAY"] = sl::VIEW_LEFT_UNRECTIFIED_GRAY;
            (*this)["VIEW_RIGHT_UNRECTIFIED_GRAY"] = sl::VIEW_RIGHT_UNRECTIFIED_GRAY;
            (*this)["VIEW_SIDE_BY_SIDE"] = sl::VIEW_SIDE_BY_SIDE;
            (*this)["VIEW_DEPTH"] = sl::VIEW_DEPTH;
            (*this)["VIEW_CONFIDENCE"] = sl::VIEW_CONFIDENCE;
            (*this)["VIEW_NORMALS"] = sl::VIEW_NORMALS;
            (*this)["VIEW_DEPTH_RIGHT"] = sl::VIEW_DEPTH_RIGHT;
            (*this)["VIEW_NORMALS_RIGHT"] = sl::VIEW_NORMALS_RIGHT;
        }
    };

    static ZEDVideoSourceMap zedVideoSourceMap;

    class ZEDDepthStreamFormatMap : public std::map<std::string, sl::DEPTH_MODE > {
    public:
        ZEDDepthStreamFormatMap() {
            (*this)["DEPTH_MODE_NONE"] = sl::DEPTH_MODE_NONE;
            (*this)["DEPTH_MODE_PERFORMANCE"] = sl::DEPTH_MODE_PERFORMANCE;
            (*this)["DEPTH_MODE_MEDIUM"] = sl::DEPTH_MODE_MEDIUM;
            (*this)["DEPTH_MODE_QUALITY"] = sl::DEPTH_MODE_QUALITY;
            (*this)["DEPTH_MODE_ULTRA"] = sl::DEPTH_MODE_ULTRA;
        }
    };

    static ZEDDepthStreamFormatMap zedDepthStreamFormatMap;

    class ZEDDepthSensingModeMap : public std::map<std::string, sl::SENSING_MODE > {
    public:
        ZEDDepthSensingModeMap() {
            (*this)["SENSING_MODE_STANDARD"] = sl::SENSING_MODE_STANDARD;
            (*this)["SENSING_MODE_FILL"] = sl::SENSING_MODE_FILL;
        }
    };

    static ZEDDepthSensingModeMap zedDepthSensingModeMap;

    class ZEDDepthMeasureSourceMap : public std::map<std::string, sl::MEASURE > {
    public:
        ZEDDepthMeasureSourceMap() {
            (*this)["MEASURE_DISPARITY"] = sl::MEASURE_DISPARITY;
            (*this)["MEASURE_DEPTH"] = sl::MEASURE_DEPTH;
            (*this)["MEASURE_CONFIDENCE"] = sl::MEASURE_CONFIDENCE;
            (*this)["MEASURE_XYZ"] = sl::MEASURE_XYZ;
            (*this)["MEASURE_XYZRGBA"] = sl::MEASURE_XYZRGBA;
            (*this)["MEASURE_XYZBGRA"] = sl::MEASURE_XYZBGRA;
            (*this)["MEASURE_XYZARGB"] = sl::MEASURE_XYZARGB;
            (*this)["MEASURE_XYZABGR"] = sl::MEASURE_XYZABGR;
            (*this)["MEASURE_NORMALS"] = sl::MEASURE_NORMALS;
            (*this)["MEASURE_DISPARITY_RIGHT"] = sl::MEASURE_DISPARITY_RIGHT;
            (*this)["MEASURE_DEPTH_RIGHT"] = sl::MEASURE_DEPTH_RIGHT;
            (*this)["MEASURE_XYZ_RIGHT"] = sl::MEASURE_XYZ_RIGHT;
            (*this)["MEASURE_XYZRGBA_RIGHT"] = sl::MEASURE_XYZRGBA_RIGHT;
            (*this)["MEASURE_XYZBGRA_RIGHT"] = sl::MEASURE_XYZBGRA_RIGHT;
            (*this)["MEASURE_XYZARGB_RIGHT"] = sl::MEASURE_XYZARGB_RIGHT;
            (*this)["MEASURE_XYZABGR_RIGHT"] = sl::MEASURE_XYZABGR_RIGHT;
            (*this)["MEASURE_NORMALS_RIGHT"] = sl::MEASURE_NORMALS_RIGHT;
        }
    };

    static ZEDDepthMeasureSourceMap zedDepthMeasureSourceMap;
}



namespace Ubitrack { namespace Drivers {
        using namespace Dataflow;


    // forward declaration
    class ZEDComponent;


/**
 * Module key for art.
 * Represents the port number on which to listen.
 */
    MAKE_NODEATTRIBUTEKEY_DEFAULT( ZEDModuleKey, int, "Camera", "zedSerialNumber", 0 );

/**
 * Component key for zed camera.
 * Represents the camera
 */
        class ZEDComponentKey
        {
        public:
            ZEDComponentKey( boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
            : m_sensor_type( ZED_SENSOR_VIDEO )
            , m_video_source( sl::VIEW_LEFT )
            , m_measure_source ( sl::MEASURE_XYZRGBA ) {
                Graph::UTQLSubgraph::EdgePtr config;

                if (subgraph->m_DataflowClass == "ZEDCameraCalibration") {
                    m_sensor_type = ZED_SENSOR_CONFIG;
                    return;
                } else if (subgraph->m_DataflowClass == "ZEDVideoStream") {
                    m_sensor_type = ZED_SENSOR_VIDEO;
                    if (subgraph->hasEdge("ImageOutput"))
                        config = subgraph->getEdge("ImageOutput");
                } else if (subgraph->m_DataflowClass == "ZEDPointCloud") {
                    m_sensor_type = ZED_SENSOR_DEPTH;
                    if (subgraph->hasEdge("PointCloudOutput"))
                        config = subgraph->getEdge("PointCloudOutput");
                } else   {
                    UBITRACK_THROW("ZED Camera Invalid Component Sensor Type.");
                }

                if (!config) {
                    UBITRACK_THROW("ZEDComponent Pattern has neither \"ImageOutput\" nor \"PointCloudOutput\" edge");
                }

                if (config->hasAttribute("zedSensorType")) {
                    std::string sSensorType = config->getAttributeString("zedSensorType");
                    if (zedSensorTypeMap.find(sSensorType) == zedSensorTypeMap.end())
                        UBITRACK_THROW("unknown sensor type: \"" + sSensorType + "\"");
                    m_sensor_type = zedSensorTypeMap[sSensorType];
                }

                if (config->hasAttribute("zedVideoSource")) {
                    std::string sVideoSource = config->getAttributeString("zedVideoSource");
                    if (zedVideoSourceMap.find(sVideoSource) == zedVideoSourceMap.end())
                        UBITRACK_THROW("unknown video source: \"" + sVideoSource + "\"");
                    m_video_source = zedVideoSourceMap[sVideoSource];
                }

                if (config->hasAttribute("zedMeasureSource")) {
                    std::string sMeasureSource = config->getAttributeString("zedMeasureSource");
                    if (zedDepthMeasureSourceMap.find(sMeasureSource) == zedDepthMeasureSourceMap.end())
                        UBITRACK_THROW("unknown Measure source: \"" + sMeasureSource + "\"");
                    m_measure_source = zedDepthMeasureSourceMap[sMeasureSource];
                }
            }


            // construct from given values
            ZEDComponentKey( ZEDSensorType  t, sl::VIEW v, sl::MEASURE m )
                    : m_sensor_type( t )
                    , m_video_source( v )
                    , m_measure_source( m )
            {}

            ZEDSensorType getSensorType() const
            {
                return m_sensor_type;
            }

            sl::VIEW getVideoSource() const
            {
                return m_video_source;
            }

            sl::MEASURE getMeasureSource() const
            {
                return m_measure_source;
            }

            // less than operator for map
            bool operator<( const ZEDComponentKey& b ) const
            {
                if ( m_sensor_type == b.m_sensor_type )
                    if ( m_video_source == b.m_video_source )
                        return m_measure_source < b.m_measure_source;
                    else
                        return m_video_source < b.m_video_source;
                else
                    return m_sensor_type < b.m_sensor_type;
            }

        protected:
            ZEDSensorType m_sensor_type;
            sl::VIEW m_video_source;
            sl::MEASURE m_measure_source;
        };


        std::ostream& operator<<( std::ostream& s, const ZEDComponentKey& k );


/**
 * Module for ZED camera.
 */
        class ZEDModule
                : public Module< ZEDModuleKey, ZEDComponentKey, ZEDModule, ZEDComponent >
        {
        public:
            /** UTQL constructor */
            ZEDModule( const ZEDModuleKey& key, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory )
            : Module< ZEDModuleKey, ZEDComponentKey, ZEDModule, ZEDComponent >(key, pFactory)
            , m_serialNumber(0)
            , m_cameraFrameRate(0)
            , m_cameraResolution(sl::RESOLUTION_HD2K)
            , m_depthMode(sl::DEPTH_MODE_ULTRA)
            , m_sensingMode(sl::SENSING_MODE_STANDARD)
            {

                Graph::UTQLSubgraph::NodePtr config;

                if (subgraph->hasNode("Camera"))
                    config = subgraph->getNode("Camera");

                if (!config) {
                    UBITRACK_THROW("ZEDModule Pattern is missing \"Camera\" node");
                }

                if (config->hasAttribute("cameraFrameRate")) {
                    config->getAttributeData("cameraFrameRate", m_cameraFrameRate);
                }

                if (config->hasAttribute("zedSerialNumber")) {
                    config->getAttributeData("zedSerialNumber", m_serialNumber);
                }

                if (config->hasAttribute("zedCameraResolution")) {
                    std::string sCameraResolution = config->getAttributeString("zedCameraResolution");
                    if (zedStreamResolutionMap.find(sCameraResolution) == zedStreamResolutionMap.end())
                        UBITRACK_THROW("unknown stream resolution: \"" + sCameraResolution + "\"");
                    m_cameraResolution = zedStreamResolutionMap[sCameraResolution];
                }

                if (config->hasAttribute("zedDepthMode")) {
                    std::string sDepthMode = config->getAttributeString("zedDepthMode");
                    if (zedDepthStreamFormatMap.find(sDepthMode) == zedDepthStreamFormatMap.end())
                        UBITRACK_THROW("unknown depth mode: \"" + sDepthMode + "\"");
                    m_depthMode = zedDepthStreamFormatMap[sDepthMode];
                }

                if (config->hasAttribute("zedSensingMode")) {
                    std::string sSensingMode = config->getAttributeString("zedSensingMode");
                    if (zedDepthSensingModeMap.find(sSensingMode) == zedDepthSensingModeMap.end())
                        UBITRACK_THROW("unknown sensing mode: \"" + sSensingMode + "\"");
                    m_sensingMode = zedDepthSensingModeMap[sSensingMode];
                }
            }

            /** destructor */
            virtual ~ZEDModule() {};

            virtual void startModule();

            void startCapturing();

            virtual void stopModule();

            void captureThread();

        protected:

            /** the associated ZED device **/
            sl::Camera m_zedcamera;

            /** camera serial **/
            unsigned int m_serialNumber;

            /** camera video-mode **/
            sl::RESOLUTION m_cameraResolution;

            /** depth-mode **/
            sl::DEPTH_MODE m_depthMode;

            /** sensing-mode **/
            sl::SENSING_MODE m_sensingMode;

            /** framerate **/
            int m_cameraFrameRate;

            /** create the components **/
            boost::shared_ptr< ZEDComponent > createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
                                                                 const ComponentKey& key, ModuleClass* pModule );
            // stop the thread?
            volatile bool m_bStop;

            // the thread
            boost::scoped_ptr< boost::thread > m_Thread;

        };


/**
 * BaseComponent for ZED sensor.
 */
        class ZEDComponent : public ZEDModule::Component {
        public:
            /** constructor */
            ZEDComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph,
                    const ZEDComponentKey& componentKey, ZEDModule* pModule )
                    : ZEDModule::Component(name, componentKey, pModule)
            {

            }

            /** initialize component **/
            virtual void init(sl::Camera &cam) {};

            /** process data **/
            virtual void process(Measurement::Timestamp ts, sl::Camera &cam) {};


            /** destructor */
            virtual ~ZEDComponent() {};

        };



/**
 * VideoComponent for ZED sensor.
 */
        class ZEDVideoComponent : public ZEDComponent {
        public:
            /** constructor */
            ZEDVideoComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph,
                          const ZEDComponentKey& componentKey, ZEDModule* pModule )
                    : ZEDComponent(name, subgraph, componentKey, pModule)
                    , m_outputPort("ImageOutput", *this)
                    , m_mat_type(sl::MAT_TYPE_8U_C4)
                    , m_imageWidth(0)
                    , m_imageHeight(0)
                    , m_autoGPUUpload( false )
            {
                Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
                if (oclManager.isEnabled()) {
                    if (subgraph->m_DataflowAttributes.hasAttribute("uploadImageOnGPU")){
                        m_autoGPUUpload = subgraph->m_DataflowAttributes.getAttributeString("uploadImageOnGPU") == "true";
                    }
                    if (m_autoGPUUpload){
                        oclManager.activate();
                    }
                }

            }

            /** initialize component **/
            virtual void init(sl::Camera &cam);

            /** process data **/
            virtual void process(Measurement::Timestamp ts, sl::Camera &cam);


            /** destructor */
            virtual ~ZEDVideoComponent() {};

        protected:
            Dataflow::PushSupplier <Measurement::ImageMeasurement> m_outputPort;
            unsigned int m_imageWidth;
            unsigned int m_imageHeight;

            sl::MAT_TYPE m_mat_type;
            std::unique_ptr<sl::Mat> m_image_zed;
            Vision::Image::ImageFormatProperties m_imageFormatProperties;
            bool m_autoGPUUpload;


        };

        class ZEDCameraCalibrationComponent : public ZEDComponent {
        public:
            /** constructor */
            ZEDCameraCalibrationComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph,
                                 const ZEDComponentKey& componentKey, ZEDModule* pModule )
                    : ZEDComponent(name, subgraph, componentKey, pModule)
                    , m_leftIntrinsicsMatrixPort("LeftIntrinsics", *this, boost::bind(&ZEDCameraCalibrationComponent::getLeftIntrinsic, this, _1))
                    , m_leftCameraIntrinsicsPort("LeftCameraModel", *this, boost::bind(&ZEDCameraCalibrationComponent::getLeftCameraModel, this, _1))
                    , m_rightIntrinsicsMatrixPort("RightIntrinsics", *this, boost::bind(&ZEDCameraCalibrationComponent::getRightIntrinsic, this, _1))
                    , m_rightCameraIntrinsicsPort("RightCameraModel", *this, boost::bind(&ZEDCameraCalibrationComponent::getRightCameraModel, this, _1))
                    , m_leftToRightTransformPort("LeftToRightTransform", *this, boost::bind(&ZEDCameraCalibrationComponent::getLeftToRightTransform, this, _1))
            {


            }

            /** initialize component **/
            virtual void init(sl::Camera &cam);

            /** process data - nothing to do here ..**/
            virtual void process(Measurement::Timestamp ts, sl::Camera &cam) {};


            /** destructor */
            virtual ~ZEDCameraCalibrationComponent() {};

        protected:

            /** handler method for incoming pull requests */
            Measurement::Matrix3x3 getLeftIntrinsic(Measurement::Timestamp t)
            {
                return Measurement::Matrix3x3(t, m_leftCameraIntrinsics.matrix);
            }

            Measurement::CameraIntrinsics getLeftCameraModel(Measurement::Timestamp t)
            {
                return Measurement::CameraIntrinsics(t, m_leftCameraIntrinsics);
            }

            Measurement::Matrix3x3 getRightIntrinsic(Measurement::Timestamp t)
            {
                return Measurement::Matrix3x3(t, m_rightCameraIntrinsics.matrix);
            }

            Measurement::CameraIntrinsics getRightCameraModel(Measurement::Timestamp t)
            {
                return Measurement::CameraIntrinsics(t, m_rightCameraIntrinsics);
            }

            Measurement::Pose getLeftToRightTransform(Measurement::Timestamp t)
            {
                return Measurement::Pose(t, m_leftToRightTransform);
            }

            Dataflow::PullSupplier <Measurement::CameraIntrinsics> m_leftCameraIntrinsicsPort;
            Dataflow::PullSupplier <Measurement::Matrix3x3> m_leftIntrinsicsMatrixPort;
            Dataflow::PullSupplier <Measurement::CameraIntrinsics> m_rightCameraIntrinsicsPort;
            Dataflow::PullSupplier <Measurement::Matrix3x3> m_rightIntrinsicsMatrixPort;
            Dataflow::PullSupplier <Measurement::Pose> m_leftToRightTransformPort;

            Math::CameraIntrinsics<double> m_leftCameraIntrinsics;
            Math::CameraIntrinsics<double> m_rightCameraIntrinsics;
            Math::Pose m_leftToRightTransform;
        };


        class ZEDPointCloudComponent : public ZEDComponent {
        public:
            /** constructor */
            ZEDPointCloudComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph,
                               const ZEDComponentKey& componentKey, ZEDModule* pModule )
                    : ZEDComponent(name, subgraph, componentKey, pModule)
                    , m_outputPort("PointCloudOutput", *this)
                    , m_mat_type(sl::MAT_TYPE_32F_C4)
                    , m_imageWidth(0)
                    , m_imageHeight(0)
                    , m_numberPoints(0)
            {

                if (componentKey.getMeasureSource() != sl::MEASURE_XYZ) {
                    UBITRACK_THROW("This component requires MeasureSource: MEASURE_XYZ");
                }

            }

            /** initialize component **/
            virtual void init(sl::Camera &cam);

            /** process data **/
            virtual void process(Measurement::Timestamp ts, sl::Camera &cam);

            /** destructor */
            virtual ~ZEDPointCloudComponent() {};
        protected:
            Dataflow::PushSupplier <Measurement::PositionList> m_outputPort;

            unsigned int m_imageWidth;
            unsigned int m_imageHeight;
            unsigned int m_numberPoints;

            sl::MAT_TYPE m_mat_type;
            std::unique_ptr<sl::Mat> m_pointcloud_zed;

        };

} } // namespace Ubitrack::Drivers

#endif //UBITRACK_DEVICE_CAMERA_ZED_ZEDCAPTURECOMPONENT_H
