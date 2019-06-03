%{

/* Includes additional C++ in the wrapper code */

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMaterialSurface.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"


#include "chrono_models/vehicle/generic/Generic_Vehicle.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_Vehicle.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_VehicleReduced.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_VehicleFull.h"

#include "chrono_models/vehicle/sedan/Sedan_Vehicle.h"


%}


//%shared_ptr(chrono::vehicle::RigidTerrain::Patch)

%import(module = "pychrono.core") "ChMaterialSurface.i"
%import "ChSuspension.i"
%import "ChDriveline.i"
%import "ChSteering.i"
%import "ChPowertrain.i"

// Model:

%include "../chrono_models/vehicle/generic/Generic_Vehicle.h"

%include "../chrono_models/vehicle/hmmwv/HMMWV_Vehicle.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_VehicleReduced.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_VehicleFull.h"

%include "../chrono_models/vehicle/sedan/Sedan_Vehicle.h"