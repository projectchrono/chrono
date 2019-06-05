%{

/* Includes additional C++ in the wrapper code */


#include "chrono_vehicle/ChVehicleModelData.h"


#include "chrono_models/ChApiModels.h"

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/vehicle/generic/Generic_BrakeSimple.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_BrakeSimple.h"
#include "chrono_models/vehicle/sedan/Sedan_BrakeSimple.h"
%}



%import "../chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

%shared_ptr(chrono::vehicle::generic::Generic_BrakeSimple)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_BrakeSimple)
%shared_ptr(chrono::vehicle::sedan::Sedan_BrakeSimple)

// Model:

%include "../chrono_models/vehicle/generic/Generic_BrakeSimple.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_BrakeSimple.h"
%include "../chrono_models/vehicle/sedan/Sedan_BrakeSimple.h"