%{

/* Includes the header in the wrapper code */
#include "chrono_sensor/sensors/ChNoiseModel.h"

using namespace chrono;
using namespace chrono::sensor;

%}

%shared_ptr(chrono::sensor::ChNoiseModel)
%shared_ptr(chrono::sensor::ChNoiseNone)
%shared_ptr(chrono::sensor::ChNoiseNormal)
%shared_ptr(chrono::sensor::ChNoiseNormalDrift)
%shared_ptr(chrono::sensor::ChNoiseRandomWalks)

%include "../../../chrono_sensor/sensors/ChNoiseModel.h"