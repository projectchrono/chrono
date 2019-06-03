%{

/* Includes additional C++ in the wrapper code */

#include "chrono/core/ChBezierCurve.h"
#include "chrono_vehicle/ChVehicle.h"

#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChSteeringController.h"
#include "chrono_vehicle/utils/ChAdaptiveSpeedController.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"


%}

//%import(module = "pychrono.core") "../chrono/core/ChBezierCurve.h"
//%shared_ptr(chrono::vehicle::RigidTerrain::Patch)

%template(vector_Entry) std::vector< chrono::vehicle::ChDataDriver::Entry >;


/* Parse the header file to generate wrappers */


%include "../chrono_vehicle/ChDriver.h"
%include "../chrono_vehicle/driver/ChDataDriver.h"
%include "../chrono_vehicle/utils/ChSpeedController.h"
%include "../chrono_vehicle/utils/ChSteeringController.h"
%include "../chrono_vehicle/utils/ChAdaptiveSpeedController.h"

%include "../chrono_vehicle/driver/ChPathFollowerDriver.h"
%include "../chrono_vehicle/driver/ChPathFollowerACCDriver.h"

//%include "models/DriverModels.i"