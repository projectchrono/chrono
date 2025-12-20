%{

/* Includes the header in the wrapper code */
#include "chrono_sensor/sensors/ChGPSSensor.h"

using namespace chrono;
using namespace chrono::sensor;

%}

%shared_ptr(chrono::sensor::ChGPSSensor)

/* Parse the header file to generate wrappers */
%include "../../../chrono_sensor/sensors/ChGPSSensor.h"
