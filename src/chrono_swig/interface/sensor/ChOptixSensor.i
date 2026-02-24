%{

/* Includes the header in the wrapper code */
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChNormalCamera.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"

using namespace chrono;
using namespace chrono::sensor;

%}

%shared_ptr(chrono::sensor::ChOptixSensor)
%shared_ptr(chrono::sensor::ChCameraSensor)
%shared_ptr(chrono::sensor::ChSegmentationCamera)
%shared_ptr(chrono::sensor::ChDepthCamera)
%shared_ptr(chrono::sensor::ChLidarSensor)
%shared_ptr(chrono::sensor::ChNormalCamera)
%shared_ptr(chrono::sensor::ChRadarSensor)
%shared_ptr(chrono::sensor::ChPhysCameraSensor)

/* Parse the header file to generate wrappers */
%include "../../../chrono_sensor/sensors/ChOptixSensor.h"
%include "../../../chrono_sensor/sensors/ChCameraSensor.h"    
%include "../../../chrono_sensor/sensors/ChSegmentationCamera.h"
%include "../../../chrono_sensor/sensors/ChDepthCamera.h"
%include "../../../chrono_sensor/sensors/ChLidarSensor.h"
%include "../../../chrono_sensor/sensors/ChNormalCamera.h"
%include "../../../chrono_sensor/sensors/ChRadarSensor.h"
%include "../../../chrono_sensor/sensors/ChPhysCameraSensor.h"

%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChCameraSensor)
%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChSegmentationCamera)
%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChDepthCamera)
%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChLidarSensor)
%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChNormalCamera)
%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChRadarSensor)
%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChPhysCameraSensor)
