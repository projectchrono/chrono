%{

/* Includes the header in the wrapper code */
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"

using namespace chrono;
using namespace chrono::sensor;

%}

%shared_ptr(chrono::sensor::ChOptixSensor)
%shared_ptr(chrono::sensor::ChCameraSensor)
%shared_ptr(chrono::sensor::ChSegmentationCamera)
%shared_ptr(chrono::sensor::ChDepthCamera)
%shared_ptr(chrono::sensor::ChLidarSensor)
%shared_ptr(chrono::sensor::ChRadarSensor)

/* Parse the header file to generate wrappers */
%include "../../../chrono_sensor/sensors/ChOptixSensor.h"
%include "../../../chrono_sensor/sensors/ChCameraSensor.h"    
%include "../../../chrono_sensor/sensors/ChSegmentationCamera.h"
%include "../../../chrono_sensor/sensors/ChDepthCamera.h"
%include "../../../chrono_sensor/sensors/ChLidarSensor.h"
%include "../../../chrono_sensor/sensors/ChRadarSensor.h"
