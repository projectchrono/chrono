%{

/* Includes additional C++ in the wrapper code */

#include <string>
#include <vector>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"


#include "chrono_vehicle/ChPowertrain.h"

#include "chrono_vehicle/powertrain/ChSimplePowertrain.h"
#include "chrono_vehicle/powertrain/ChSimpleMapPowertrain.h"
#include "chrono_vehicle/powertrain/ChSimpleCVTPowertrain.h"
#include "chrono_vehicle/powertrain/ChShaftsPowertrain.h"

#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/powertrain/SimpleMapPowertrain.h"
#include "chrono_vehicle/powertrain/SimpleCVTPowertrain.h"
#include "chrono_vehicle/powertrain/ShaftsPowertrain.h"


#include "chrono_models/vehicle/generic/Generic_SimplePowertrain.h"
#include "chrono_models/vehicle/generic/Generic_SimpleMapPowertrain.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_SimplePowertrain.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_SimpleMapPowertrain.h"
#include "chrono_models/vehicle/sedan/Sedan_SimpleMapPowertrain.h"

%}

%shared_ptr(chrono::vehicle::ChPowertrain)

%shared_ptr(chrono::vehicle::ChSimplePowertrain)
%shared_ptr(chrono::vehicle::ChSimpleMapPowertrain)
%shared_ptr(chrono::vehicle::ChSimpleCVTPowertrain)
%shared_ptr(chrono::vehicle::ChShaftsPowertrain)

%shared_ptr(chrono::vehicle::SimplePowertrain)
%shared_ptr(chrono::vehicle::SimpleMapPowertrain)
%shared_ptr(chrono::vehicle::SimpleCVTPowertrain)
%shared_ptr(chrono::vehicle::ShaftsPowertrain)

%import "../chrono_vehicle/ChPart.h"
%import(module = "pychrono.core") "ChShaft.i"

/* Parse the header file to generate wrappers */

%include "../chrono_vehicle/ChPowertrain.h"

%include "../chrono_vehicle/powertrain/ChSimplePowertrain.h"
%include "../chrono_vehicle/powertrain/ChSimpleMapPowertrain.h"
%include "../chrono_vehicle/powertrain/ChSimpleCVTPowertrain.h"
%include "../chrono_vehicle/powertrain/ChShaftsPowertrain.h"

%include "../chrono_vehicle/powertrain/SimplePowertrain.h"
%include "../chrono_vehicle/powertrain/SimpleMapPowertrain.h"
%include "../chrono_vehicle/powertrain/SimpleCVTPowertrain.h"
%include "../chrono_vehicle/powertrain/ShaftsPowertrain.h"


%include "models/PowertrainModels.i"