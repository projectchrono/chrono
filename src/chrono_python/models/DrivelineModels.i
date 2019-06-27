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

#include "chrono_models/vehicle/generic/Generic_Driveline2WD.h"
#include "chrono_models/vehicle/generic/Generic_SimpleDriveline.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_Driveline2WD.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Driveline4WD.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_SimpleDriveline.h"

#include "chrono_models/vehicle/sedan/Sedan_Driveline2WD.h"

%}


%shared_ptr(chrono::vehicle::generic::Generic_Driveline2WD)
%shared_ptr(chrono::vehicle::generic::Generic_SimpleDriveline)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Driveline2WD)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Driveline4WD)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_SimpleDriveline)
%shared_ptr(chrono::vehicle::sedan::Sedan_Driveline2WD)


/* Parse the header file to generate wrappers */
%import "ChDriveline.i"

// Model:
%include "../chrono_models/vehicle/generic/Generic_Driveline2WD.h"
%include "../chrono_models/vehicle/generic/Generic_SimpleDriveline.h"

%include "../chrono_models/vehicle/hmmwv/HMMWV_Driveline2WD.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_Driveline4WD.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_SimpleDriveline.h"

%include "../chrono_models/vehicle/sedan/Sedan_Driveline2WD.h"