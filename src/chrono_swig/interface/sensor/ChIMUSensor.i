%{

/* Includes the header in the wrapper code */
#include "chrono_sensor/sensors/ChIMUSensor.h"

using namespace chrono;
using namespace chrono::sensor;

%}

%shared_ptr(chrono::sensor::ChAccelerometerSensor)
%shared_ptr(chrono::sensor::ChGyroscopeSensor)
%shared_ptr(chrono::sensor::ChMagnetometerSensor)

/* Parse the header file to generate wrappers */
%include "../../../chrono_sensor/sensors/ChIMUSensor.h"
