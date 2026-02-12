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
