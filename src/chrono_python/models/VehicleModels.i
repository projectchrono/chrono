%{

/* Includes additional C++ in the wrapper code */

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMaterialSurface.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"


#include "chrono_models/vehicle/generic/Generic_Vehicle.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Vehicle.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_VehicleReduced.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_VehicleFull.h"

#include "chrono_models/vehicle/sedan/Sedan.h"
#include "chrono_models/vehicle/sedan/Sedan_Vehicle.h"


%}


%shared_ptr(chrono::vehicle::generic::Generic_Vehicle)

%shared_ptr(chrono::vehicle::hmmwv::HMMWV)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Vehicle)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_VehicleReduced)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_VehicleFull)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Reduced)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Full)

%shared_ptr(chrono::vehicle::sedan::Sedan)
%shared_ptr(chrono::vehicle::sedan::Sedan_Vehicle)


%import(module = "pychrono.core") "ChMaterialSurface.i"
%import "ChSuspension.i"
%import "ChDriveline.i"
%import "ChSteering.i"
%import "ChPowertrain.i"
%import "ChChassis.i"
%import "ChTire.i"
%import "../chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
%import "../chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
%import "../chrono_vehicle/ChVehicle.h"

// Model:

%include "../chrono_models/vehicle/generic/Generic_Vehicle.h"

%include "../chrono_models/vehicle/hmmwv/HMMWV.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_Vehicle.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_VehicleReduced.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_VehicleFull.h"

%include "../chrono_models/vehicle/sedan/Sedan.h"
%include "../chrono_models/vehicle/sedan/Sedan_Vehicle.h"