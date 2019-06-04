%{

/* Includes additional C++ in the wrapper code */

#include <string>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"
#include "chrono_vehicle/wheeled_vehicle/steering/ChPitmanArm.h"
#include "chrono_vehicle/wheeled_vehicle/steering/ChPitmanArmShafts.h"
#include "chrono_vehicle/wheeled_vehicle/steering/ChRackPinion.h"

#include "chrono_vehicle/wheeled_vehicle/steering/ChRotaryArm.h"
#include "chrono_vehicle/wheeled_vehicle/steering/PitmanArm.h"
#include "chrono_vehicle/wheeled_vehicle/steering/RackPinion.h"
#include "chrono_vehicle/wheeled_vehicle/steering/RotaryArm.h"


#include "chrono_thirdparty/rapidjson/document.h"

%}
%import(module = "pychrono.core") "ChShaft.i"
%import "../chrono_vehicle/ChPart.h"
//TODO: import once it is done!!!!
//%import "ChSuspensions.i"


//%shared_ptr(chrono::vehicle::RigidTerrain::Patch)

/* Parse the header file to generate wrappers */

%include "../chrono_vehicle/wheeled_vehicle/ChSteering.h"
%ignore chrono::vehicle::ChPitmanArm::getLocation;
%ignore chrono::vehicle::ChPitmanArm::getDirection;
%include "../chrono_vehicle/wheeled_vehicle/steering/ChPitmanArm.h"
%include "../chrono_vehicle/wheeled_vehicle/steering/ChPitmanArmShafts.h"
%include "../chrono_vehicle/wheeled_vehicle/steering/ChRackPinion.h"
%ignore chrono::vehicle::ChRotaryArm::getLocation;
%ignore chrono::vehicle::ChRotaryArm::getDirection;
%include "../chrono_vehicle/wheeled_vehicle/steering/ChRotaryArm.h"
%include "../chrono_vehicle/wheeled_vehicle/steering/PitmanArm.h"
%include "../chrono_vehicle/wheeled_vehicle/steering/RackPinion.h"
%include "../chrono_vehicle/wheeled_vehicle/steering/RotaryArm.h"
