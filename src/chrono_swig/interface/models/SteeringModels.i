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

#include "chrono_models/vehicle/hmmwv/steering/HMMWV_PitmanArm.h"
#include "chrono_models/vehicle/hmmwv/steering/HMMWV_RackPinion.h"

#include "chrono_models/vehicle/citybus/CityBus_RotaryArm.h"
#include "chrono_models/vehicle/citybus/CityBus_RackPinion.h"

#include "chrono_models/vehicle/man/MAN_5t_RotaryArm.h"
#include "chrono_models/vehicle/man/MAN_10t_RotaryArm2.h"

#include "chrono_models/vehicle/uaz/UAZBUS_RotaryArm.h"

#include "chrono_models/vehicle/gator/Gator_RackPinion.h"

#include "chrono_models/vehicle/artcar/ARTcar_PitmanArm.h"

#include "chrono_models/vehicle/feda/FEDA_PitmanArm.h"
%}

%shared_ptr(chrono::vehicle::hmmwv::HMMWV_PitmanArm)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_RackPinion)

%shared_ptr(chrono::vehicle::citybus::CityBus_RotaryArm)
%shared_ptr(chrono::vehicle::citybus::CityBus_RackPinion)

%shared_ptr(chrono::vehicle::man::MAN_5t_RotaryArm)
%shared_ptr(chrono::vehicle::man::MAN_10t_RotaryArm2)

%shared_ptr(chrono::vehicle::uaz::UAZBUS_RotaryArm)

%shared_ptr(chrono::vehicle::artcar::ARTcar_PitmanArm)

%shared_ptr(chrono::vehicle::gator::Gator_RackPinion)

%shared_ptr(chrono::vehicle::feda::FEDA_PitmanArm)

/* Parse the header file to generate wrappers */
%import "chrono_swig/interface/vehicle/ChSteering.i"

// Model:

%include "../../../chrono_models/vehicle/hmmwv/steering/HMMWV_PitmanArm.h"
%include "../../../chrono_models/vehicle/hmmwv/steering/HMMWV_RackPinion.h"

%include "../../../chrono_models/vehicle/citybus/CityBus_RotaryArm.h"
%include "../../../chrono_models/vehicle/citybus/CityBus_RackPinion.h"

%include "../../../chrono_models/vehicle/man/MAN_5t_RotaryArm.h"
%include "../../../chrono_models/vehicle/man/MAN_10t_RotaryArm2.h"

%include "../../../chrono_models/vehicle/uaz/UAZBUS_RotaryArm.h"

%include "../../../chrono_models/vehicle/gator/Gator_RackPinion.h"

%include "../../../chrono_models/vehicle/artcar/ARTcar_PitmanArm.h"

%include "../../../chrono_models/vehicle/feda/FEDA_PitmanArm.h"
