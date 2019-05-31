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
#include "chrono/physics/ChShaft.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

#include "chrono_vehicle/wheeled_vehicle/ChDriveline.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ShaftsDriveline2WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/SimpleDriveline.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ShaftsDriveline4WD.h"

#include "chrono_thirdparty/rapidjson/document.h"


%}
%import(module = "pychrono.core") "ChShaft.i"
//TODO: import these once it is done!!!!
//%import "ChSuspension.i"


//%shared_ptr(chrono::vehicle::RigidTerrain::Patch)

/* Parse the header file to generate wrappers */
%include "../chrono_vehicle/wheeled_vehicle/ChDriveline.h"
%include "../chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"
%include "../chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"
%include "../chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline.h"
%include "../chrono_vehicle/wheeled_vehicle/driveline/ShaftsDriveline2WD.h"
%include "../chrono_vehicle/wheeled_vehicle/driveline/SimpleDriveline.h"
%include "../chrono_vehicle/wheeled_vehicle/driveline/ShaftsDriveline4WD.h"
%include "models/DrivelineModels.i"