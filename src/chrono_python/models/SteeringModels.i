%{

/* Includes additional C++ in the wrapper code */

#include <string>
#include <vector>
#include "chrono/core/ChVector.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono/physics/ChShaft.h"
#include "chrono_vehicle/ChPart.h"

#include "chrono_models/vehicle/citybus/CityBus_RotaryArm.h"
#include "chrono_models/vehicle/citybus/CityBus_RackPinion.h"

%}

%shared_ptr(chrono::vehicle::citybus::CityBus_RotaryArm)
%shared_ptr(chrono::vehicle::citybus::CityBus_RackPinion)


/* Parse the header file to generate wrappers */
%import "ChSteering.i"

// Model:

%include "../chrono_models/vehicle/citybus/CityBus_RotaryArm.h"
%include "../chrono_models/vehicle/citybus/CityBus_RackPinion.h"
