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

%}


%shared_ptr(chrono::vehicle::generic::Generic_Chassis)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Chassis)
%shared_ptr(chrono::vehicle::sedan::Sedan_Chassis)

/* Parse the header file to generate wrappers */
%import "ChChassis.i"
%import(module = "pychrono.core") "../chrono/assets/ChAsset.h"
%import(module = "pychrono.core") "../chrono/assets/ChAssetLevel.h"

// Model:
%include "../chrono_models/vehicle/generic/Generic_Chassis.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_Chassis.h"
%include "../chrono_models/vehicle/sedan/Sedan_Chassis.h"