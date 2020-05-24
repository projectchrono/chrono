%{

/* Includes additional C++ in the wrapper code */
#include <string>
#include <vector>

#include "chrono_vehicle/utils/ChSteeringController.h"
#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChAdaptiveSpeedController.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
//#include "chrono_vehicle/utils/ChUtilsJSON.h"

%}


%import "ChPowertrain.i"
%import "ChChassis.i"
%import "ChSteering.i"
%import "../../chrono_vehicle/ChVehicle.h"


/* Parse the header file to generate wrappers */


%include "../../chrono_vehicle/utils/ChSteeringController.h"
%include "../../chrono_vehicle/utils/ChSpeedController.h"
%include "../../chrono_vehicle/utils/ChAdaptiveSpeedController.h"
%include "../../chrono_vehicle/utils/ChVehiclePath.h"
//%include "../../chrono_vehicle/utils/ChUtilsJSON.h"
