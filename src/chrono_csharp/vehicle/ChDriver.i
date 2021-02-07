%{

/* Includes additional C++ in the wrapper code */
#include <string>
#include <vector>

#include "chrono/ChConfig.h"

#include "chrono/core/ChBezierCurve.h"
#include "chrono_vehicle/ChVehicle.h"

#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChSteeringController.h"
#include "chrono_vehicle/utils/ChAdaptiveSpeedController.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"

//to import/wrap
#include "chrono/utils/ChUtilsChaseCamera.h"

%}


%rename(DriverInputs) chrono::vehicle::ChDriver::Inputs;


%import "../chrono/core/ChBezierCurve.h"

%import "ChPowertrain.i"
%import "ChChassis.i"
%import "../../chrono_vehicle/ChVehicle.h"

%rename(DataDriverEntry) chrono::vehicle::ChDataDriver::Entry;
%template(vector_Entry) std::vector< chrono::vehicle::ChDataDriver::Entry >;


/* Parse the header file to generate wrappers */
%include "../../chrono_vehicle/ChDriver.h"
%include "../../chrono_vehicle/driver/ChDataDriver.h"
%include "../../chrono_vehicle/utils/ChSpeedController.h"
%include "../../chrono_vehicle/utils/ChSteeringController.h"
%include "../../chrono_vehicle/utils/ChAdaptiveSpeedController.h"
%include "../../chrono_vehicle/driver/ChPathFollowerDriver.h"
%include "../../chrono_vehicle/driver/ChPathFollowerACCDriver.h"
%include "../chrono/utils/ChUtilsChaseCamera.h"
