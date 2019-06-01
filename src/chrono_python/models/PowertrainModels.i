%{

/* Includes additional C++ in the wrapper code */

#include <string>
#include <vector>

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChPowertrain.h"
#include "chrono/core/ChCubicSpline.h"

//#include "chrono_models/ChApiModels.h"

#include "chrono_models/vehicle/generic/Generic_SimplePowertrain.h"
#include "chrono_models/vehicle/generic/Generic_SimpleMapPowertrain.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_SimplePowertrain.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_SimpleMapPowertrain.h"

#include "chrono_models/vehicle/sedan/Sedan_SimpleMapPowertrain.h"


%}




%import "ChPowertrain.i"

// Model:
%include "../chrono_models/vehicle/generic/Generic_SimplePowertrain.h"
%include "../chrono_models/vehicle/generic/Generic_SimpleMapPowertrain.h"

%include "../chrono_models/vehicle/hmmwv/HMMWV_SimplePowertrain.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_SimpleMapPowertrain.h"

%include "../chrono_models/vehicle/sedan/Sedan_SimpleMapPowertrain.h"