%{

/* Includes additional C++ in the wrapper code */

#include <string>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

#include "chrono_vehicle/wheeled_vehicle/ChSubchassis.h"

#include "chrono_vehicle/wheeled_vehicle/subchassis/ChBalancer.h"

#include "chrono_vehicle/wheeled_vehicle/subchassis/Balancer.h"

#include "chrono_thirdparty/rapidjson/document.h"
%}

%shared_ptr(chrono::vehicle::ChSubchassis)
%shared_ptr(chrono::vehicle::ChBalancer)
%shared_ptr(chrono::vehicle::Balancer)

%import "../../../chrono_vehicle/ChPart.h"

/* Parse the header file to generate wrappers */

%include "../../../chrono_vehicle/wheeled_vehicle/ChSubchassis.h"
%ignore chrono::vehicle::ChBalancer::GetLocation;
%include "../../../chrono_vehicle/wheeled_vehicle/subchassis/ChBalancer.h"
%include "../../../chrono_vehicle/wheeled_vehicle/subchassis/Balancer.h"

%include "chrono_swig/interface/models/SubchassisModels.i"

