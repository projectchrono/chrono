%{

/* Includes additional C++ in the wrapper code */

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_models/ChApiModels.h"

#include "chrono_models/vehicle/generic/Generic_Wheel.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"

#include "chrono_models/vehicle/sedan/Sedan_Wheel.h"
%}



%import "../chrono_vehicle/wheeled_vehicle/ChWheel.h"

// Model:

%include "../chrono_models/vehicle/generic/Generic_Wheel.h"

%include "../chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"

%include "../chrono_models/vehicle/sedan/Sedan_Wheel.h"