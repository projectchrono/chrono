%{
#include <string>
#include <vector>

#include "chrono_vehicle/utils/ChSteeringController.h"
#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChAdaptiveSpeedController.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
%}

%shared_ptr(chrono::vehicle::StraightLinePath)
%shared_ptr(chrono::vehicle::CirclePath)
%shared_ptr(chrono::vehicle::DoubleLaneChangePath)

%import "../../../chrono_vehicle/ChVehicle.h"

// Parse the header file to generate wrappers
%include "../../../chrono_vehicle/utils/ChSteeringController.h"
%include "../../../chrono_vehicle/utils/ChSpeedController.h"
%include "../../../chrono_vehicle/utils/ChAdaptiveSpeedController.h"
%include "../../../chrono_vehicle/utils/ChVehiclePath.h"
%include "../../../chrono_vehicle/utils/ChUtilsJSON.h"
