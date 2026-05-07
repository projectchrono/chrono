%{

/* Includes the header in the wrapper code */
#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/sensors/Sensor.h"

using namespace chrono;
using namespace chrono::sensor;

%}

%shared_ptr(chrono::sensor::ChSensor)
%shared_ptr(chrono::sensor::ChDynamicSensor)
%shared_ptr(chrono::sensor::Sensor)

/* Parse the header file to generate wrappers */
%include "../../../chrono_sensor/sensors/ChSensor.h"
%include "../../../chrono_sensor/sensors/Sensor.h"

#ifdef CHRONO_HAS_OPTIX
  %template(GetMostRecentRGBA8Buffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>>> > ;
  %template(GetMostRecentR8Buffer)    chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::SensorBufferT<std::shared_ptr<char[]>>> > ;

  %template(GetMostRecentDIBuffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>>> > ;
  %template(GetMostRecentXYZIBuffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>>> > ;

  %template(GetMostRecentRadarBuffer) chrono::sensor::ChSensor::GetMostRecentBuffer<std::shared_ptr<chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarReturn[]>>>>;
  %template(GetMostRecentRadarXYZBuffer) chrono::sensor::ChSensor::GetMostRecentBuffer<std::shared_ptr<chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarXYZReturn[]>>>>;

  %template(GetMostRecentDepthBuffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDepth[]>>> > ;

#endif

%template(GetMostRecentAccelBuffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::AccelData[]>>> > ;
%template(GetMostRecentGyroBuffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GyroData[]>>> > ;
%template(GetMostRecentMagnetBuffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::MagnetData[]>>> > ;
%template(GetMostRecentGPSBuffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>>> > ;
