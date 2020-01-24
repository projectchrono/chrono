%{

/* Includes additional C++ in the wrapper code */

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

#include "chrono_models/vehicle/citybus/CityBus_RotaryArm.h"
#include "chrono_models/vehicle/citybus/CityBus_RackPinion.h"

#include "chrono_models/vehicle/man/MAN_5t_RotaryArm.h"
#include "chrono_models/vehicle/man/MAN_10t_RotaryArm2.h"
%}

%shared_ptr(chrono::vehicle::citybus::CityBus_RotaryArm)
%shared_ptr(chrono::vehicle::citybus::CityBus_RackPinion)

%shared_ptr(chrono::vehicle::man::MAN_5t_RotaryArm)
%shared_ptr(chrono::vehicle::man::MAN_10t_RotaryArm2)

/* Parse the header file to generate wrappers */
%import "ChSteering.i"

// Model:

%include "../chrono_models/vehicle/citybus/CityBus_RotaryArm.h"
%include "../chrono_models/vehicle/citybus/CityBus_RackPinion.h"

%include "../chrono_models/vehicle/man/MAN_5t_RotaryArm.h"
%include "../chrono_models/vehicle/man/MAN_10t_RotaryArm2.h"
