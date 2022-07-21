%{

/* Includes additional C++ in the wrapper code */

#include <string>
#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/chassis/RigidChassis.h"
#include "chrono_vehicle/chassis/ChRigidChassis.h"

#include "chrono_thirdparty/rapidjson/document.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include "chrono_models/vehicle/generic/Generic_Chassis.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Chassis.h"
#include "chrono_models/vehicle/sedan/Sedan_Chassis.h"
#include "chrono_models/vehicle/citybus/CityBus_Chassis.h"
#include "chrono_models/vehicle/man/MAN_5t_Chassis.h"
#include "chrono_models/vehicle/man/MAN_7t_Chassis.h"
#include "chrono_models/vehicle/man/MAN_10t_Chassis.h"
#include "chrono_models/vehicle/uaz/UAZBUS_Chassis.h"
#include "chrono_models/vehicle/gator/Gator_Chassis.h"
#include "chrono_models/vehicle/rccar/RCCar_Chassis.h"
#include "chrono_models/vehicle/feda/FEDA_Chassis.h"

#include "chrono_models/vehicle/m113/M113_Chassis.h"
%}


%shared_ptr(chrono::vehicle::generic::Generic_Chassis)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Chassis)
%shared_ptr(chrono::vehicle::sedan::Sedan_Chassis)
%shared_ptr(chrono::vehicle::citybus::CityBus_Chassis)
%shared_ptr(chrono::vehicle::man::MAN_5t_Chassis)
%shared_ptr(chrono::vehicle::man::MAN_7t_Chassis)
%shared_ptr(chrono::vehicle::man::MAN_10t_Chassis)
%shared_ptr(chrono::vehicle::uaz::UAZBUS_Chassis)
%shared_ptr(chrono::vehicle::gator::Gator_Chassis)
%shared_ptr(chrono::vehicle::rccar::RCCar_Chassis)
%shared_ptr(chrono::vehicle::feda::FEDA_Chassis)

%shared_ptr(chrono::vehicle::m113::M113_Chassis)

// Parse the header file to generate wrappers
%import "chrono_swig/interface/vehicle/ChChassis.i"

// Model:
%include "../../../chrono_models/vehicle/generic/Generic_Chassis.h"
%include "../../../chrono_models/vehicle/hmmwv/HMMWV_Chassis.h"
%include "../../../chrono_models/vehicle/sedan/Sedan_Chassis.h"
%include "../../../chrono_models/vehicle/citybus/CityBus_Chassis.h"
%include "../../../chrono_models/vehicle/man/MAN_5t_Chassis.h"
%include "../../../chrono_models/vehicle/man/MAN_7t_Chassis.h"
%include "../../../chrono_models/vehicle/man/MAN_10t_Chassis.h"
%include "../../../chrono_models/vehicle/uaz/UAZBUS_Chassis.h"
%include "../../../chrono_models/vehicle/gator/Gator_Chassis.h"
%include "../../../chrono_models/vehicle/rccar/RCCar_Chassis.h"
%include "../../../chrono_models/vehicle/feda/FEDA_Chassis.h"

%include "../../../chrono_models/vehicle/m113/M113_Chassis.h"
