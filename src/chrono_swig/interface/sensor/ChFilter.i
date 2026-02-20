%{

/* Includes the header in the wrapper code */
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGPSUpdate.h"
#include "chrono_sensor/filters/ChFilterIMUUpdate.h"
#include "chrono_sensor/filters/ChFilterTachometerUpdate.h"

#ifdef CHRONO_HAS_OPTIX

  #include "chrono_sensor/optix/ChFilterOptixRender.h"
  #include "chrono_sensor/filters/ChFilterPCfromDepth.h"
  #include "chrono_sensor/filters/ChFilterGrayscale.h"
  #include "chrono_sensor/filters/ChFilterRadarProcess.h"
  #include "chrono_sensor/filters/ChFilterRadarXYZReturn.h"
  #include "chrono_sensor/filters/ChFilterRadarSavePC.h"
  #include "chrono_sensor/filters/ChFilterRadarVisualizeCluster.h"
  #include "chrono_sensor/filters/ChFilterRadarXYZVisualize.h"
  #include "chrono_sensor/filters/ChFilterCameraNoise.h"
  #include "chrono_sensor/filters/ChFilterCameraExposure.h"

  #include "chrono_sensor/filters/ChFilterPhysCameraAggregator.h"
  #include "chrono_sensor/filters/ChFilterPhysCameraDefocusBlur.h"
  #include "chrono_sensor/filters/ChFilterPhysCameraExpsrToDV.h"
  #include "chrono_sensor/filters/ChFilterPhysCameraNoise.h"
  #include "chrono_sensor/filters/ChFilterPhysCameraVignetting.h"

  #include "chrono_sensor/filters/ChFilterSave.h"
  #include "chrono_sensor/filters/ChFilterSavePtCloud.h"
  #include "chrono_sensor/filters/ChFilterVisualize.h"
  #include "chrono_sensor/filters/ChFilterImageOps.h"
  #include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
  
#endif

using namespace chrono;
using namespace chrono::sensor;

%}


%shared_ptr(chrono::sensor::ChFilter)
%shared_ptr(chrono::sensor::ChFilterAccelerometerUpdate)
%shared_ptr(chrono::sensor::ChFilterGyroscopeUpdate)
%shared_ptr(chrono::sensor::ChFilterMagnetometerUpdate)
%shared_ptr(chrono::sensor::ChFilterGPSUpdate)

%shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< char [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< char [] > > > > )
%shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::AccelData [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::AccelData [] > > > > )
%shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::GyroData [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::GyroData [] > > > > )
%shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::MagnetData [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::MagnetData [] > > > > )
%shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::GPSData [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::GPSData [] > > > > )

#ifdef CHRONO_HAS_OPTIX

  %shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::LidarBufferT< std::shared_ptr< chrono::sensor::PixelXYZI [] > >,std::shared_ptr< chrono::sensor::LidarBufferT< std::shared_ptr< chrono::sensor::PixelXYZI [] > > > > )
  %shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::LidarBufferT< std::shared_ptr< chrono::sensor::PixelDI [] > >,std::shared_ptr< chrono::sensor::LidarBufferT< std::shared_ptr< chrono::sensor::PixelDI [] > > > > )
  %shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::RadarBufferT< std::shared_ptr< chrono::sensor::RadarReturn [] > >,std::shared_ptr< chrono::sensor::RadarBufferT< std::shared_ptr< chrono::sensor::RadarReturn[] > > > > )
  %shared_ptr(chrono::sensor::ChFilterAccess<chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarXYZReturn[]>>, std::shared_ptr<chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarXYZReturn[]>>>>)
  %shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::PixelRGBA8 [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::PixelRGBA8 [] > > > > )
  %shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::PixelDepth [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::PixelDepth [] > > > > )
  %shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::PixelSemantic [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::PixelSemantic [] > > > > )
  
  %shared_ptr(chrono::sensor::ChFilterVisualizePointCloud)
  %shared_ptr(chrono::sensor::ChFilterVisualize)
  %shared_ptr(chrono::sensor::ChFilterSave)
  %shared_ptr(chrono::sensor::ChFilterSavePtCloud)
  %shared_ptr(chrono::sensor::ChFilterOptixRender)
  %shared_ptr(chrono::sensor::ChFilterPCfromDepth)
  %shared_ptr(chrono::sensor::ChFilterGrayscale)
  %shared_ptr(chrono::sensor::ChFilterRadarProcess)
  %shared_ptr(chrono::sensor::ChFilterRadarXYZReturn)
  %shared_ptr(chrono::sensor::ChFilterRadarXYZVisualize)
  %shared_ptr(chrono::sensor::ChFilterRadarVisualizeCluster)
  %shared_ptr(chrono::sensor::ChFilterRadarSavePC)
  
  %shared_ptr(chrono::sensor::ChFilterCameraNoiseConstNormal)
  %shared_ptr(chrono::sensor::ChFilterCameraNoisePixDep)
  %shared_ptr(chrono::sensor::ChFilterCameraExposureCorrect)

  %shared_ptr(chrono::sensor::ChFilterPhysCameraAggregator)
  %shared_ptr(chrono::sensor::ChFilterPhysCameraDefocusBlur)
  %shared_ptr(chrono::sensor::ChFilterPhysCameraExpsrToDV)
  %shared_ptr(chrono::sensor::ChFilterPhysCameraNoise)
  %shared_ptr(chrono::sensor::ChFilterPhysCameraVignetting)
  
  %shared_ptr(chrono::sensor::ChFilterImageHalf4ToRGBA8)
  %shared_ptr(chrono::sensor::ChFilterRGBDHalf4ToImageHalf4)
  %shared_ptr(chrono::sensor::ChFilterRGBDHalf4ToR8)
  %shared_ptr(chrono::sensor::ChFilterImageHalf4ToRGBA16)
  %shared_ptr(chrono::sensor::ChFilterDepthToRGBA8)
  %shared_ptr(chrono::sensor::ChFilterNormalToRGBA8)
  %shared_ptr(chrono::sensor::ChFilterImageResize)
  %shared_ptr(chrono::sensor::ChFilterImgAlias)

  %shared_ptr(chrono::sensor::ChFilterCameraNoiseConstNormal)
  %shared_ptr(chrono::sensor::ChFilterCameraNoisePixDep)
#endif

%include "chrono_sensor/filters/ChFilter.h"
%include "chrono_sensor/filters/ChFilterAccess.h"
%include "chrono_sensor/filters/ChFilterGPSUpdate.h"
%include "chrono_sensor/filters/ChFilterIMUUpdate.h"


#ifdef CHRONO_HAS_OPTIX

  %include "chrono_sensor/filters/ChFilterGrayscale.h"
  %include "chrono_sensor/optix/ChFilterOptixRender.h"
  %include "chrono_sensor/filters/ChFilterPCfromDepth.h"
  %include "chrono_sensor/filters/ChFilterSave.h"
  %include "chrono_sensor/filters/ChFilterSavePtCloud.h"
  %include "chrono_sensor/filters/ChFilterVisualize.h"
  %include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
  %include "chrono_sensor/filters/ChFilterImageOps.h"
  %include "chrono_sensor/filters/ChFilterRadarProcess.h"
  %include "chrono_sensor/filters/ChFilterRadarSavePC.h"
  %include "chrono_sensor/filters/ChFilterRadarVisualizeCluster.h"
  %include "chrono_sensor/filters/ChFilterRadarXYZReturn.h"
  %include "chrono_sensor/filters/ChFilterRadarXYZVisualize.h"
  %include "chrono_sensor/filters/ChFilterCameraNoise.h"
  %include "chrono_sensor/filters/ChFilterCameraExposure.h"

  %include "chrono_sensor/filters/ChFilterPhysCameraAggregator.h"
  %include "chrono_sensor/filters/ChFilterPhysCameraDefocusBlur.h"
  %include "chrono_sensor/filters/ChFilterPhysCameraExpsrToDV.h"
  %include "chrono_sensor/filters/ChFilterPhysCameraNoise.h"
  %include "chrono_sensor/filters/ChFilterPhysCameraVignetting.h"
  
  %include "chrono_sensor/optix/scene/ChScene.h"
  %include "chrono_sensor/optix/ChOptixDefinitions.h"
  %include "chrono_sensor/optix/ChOptixUtils.h"

#endif


// Filter acces templates instances

#ifdef CHRONO_HAS_OPTIX

  // camera
  %template(ChFilterRGBA8Access)      chrono::sensor::ChFilterAccess<  chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>>,    std::shared_ptr< chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>>    >  >;
  %template(ChFilterR8Access)         chrono::sensor::ChFilterAccess<  chrono::sensor::SensorBufferT<std::shared_ptr<char[]>>,                          std::shared_ptr< chrono::sensor::SensorBufferT<std::shared_ptr<char[]>>                          >  >;
  %template(ChFilterSemanticAccess)   chrono::sensor::ChFilterAccess<  chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelSemantic[]>>, std::shared_ptr< chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelSemantic[]>> >  >;
    
  //lidar 
  %template(ChFilterDIAccess) chrono::sensor::ChFilterAccess<chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>>, std::shared_ptr<chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>>>>;
  %template(ChFilterXYZIAccess) chrono::sensor::ChFilterAccess<chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>>, std::shared_ptr<chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>>>>;
  
  //radar
  %template(ChFilterRadarAccess) chrono::sensor::ChFilterAccess<chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarReturn[]>>, std::shared_ptr<chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarReturn[]>>>>;
  %template(ChFilterRadarXYZAccess) chrono::sensor::ChFilterAccess<chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarXYZReturn[]>>, std::shared_ptr<chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarXYZReturn[]>>>>;
  
  //depth camera
  %template(ChFilterDepthAccess) chrono::sensor::ChFilterAccess<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDepth[]>>, std::shared_ptr<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDepth[]>>>>;

#endif

//dynamic 
%template(ChFilterAccelAccess) chrono::sensor::ChFilterAccess<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::AccelData[]>>, std::shared_ptr<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::AccelData[]>>>>;
%template(ChFilterGyroAccess) chrono::sensor::ChFilterAccess<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GyroData[]>>, std::shared_ptr<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GyroData[]>>>>;
%template(ChFilterMagnetAccess) chrono::sensor::ChFilterAccess<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::MagnetData[]>>, std::shared_ptr<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::MagnetData[]>>>>;
%template(ChFilterGPSAccess) chrono::sensor::ChFilterAccess<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>>, std::shared_ptr<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>>>>;
