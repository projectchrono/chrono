%{

/* Includes additional C++ in the wrapper code */

#include <string>
#include <vector>

#include "chrono/core/ChVector.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/chassis/RigidChassis.h"
#include "chrono_vehicle/chassis/ChRigidChassis.h"

#include "chrono_thirdparty/rapidjson/document.h"


%}

%shared_ptr(chrono::vehicle::ChChassis)
%shared_ptr(chrono::vehicle::ChRigidChassis)
%shared_ptr(chrono::vehicle::RigidChassis)


%import(module = "pychrono.core") "../chrono/assets/ChAsset.h"
%import(module = "pychrono.core") "../chrono/assets/ChAssetLevel.h"


/* Parse the header file to generate wrappers */
%include "../chrono_vehicle/ChChassis.h"
%include "../chrono_vehicle/chassis/ChRigidChassis.h"
%include "../chrono_vehicle/chassis/RigidChassis.h"
%include "models/ChassisModels.i"