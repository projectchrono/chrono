%{

/* Includes additional C++ in the wrapper code */

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

#include "chrono_vehicle/wheeled_vehicle/ChSubchassis.h"
#include "chrono_vehicle/wheeled_vehicle/subchassis/ChBalancer.h"
#include "chrono_vehicle/wheeled_vehicle/subchassis/Balancer.h"

#include "chrono_models/vehicle/mtv/MTV_Balancer.h"
%}

%shared_ptr(chrono::vehicle::fmtv::MTV_Balancer)

/* Parse the header file to generate wrappers */
%import "chrono_swig/interface/vehicle/ChSubchassis.i"

// Model:

%include "../../../chrono_models/vehicle/mtv/MTV_Balancer.h"
