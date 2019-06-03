%{

/* Includes additional C++ in the wrapper code */

#include "chrono/core/ChBezierCurve.h"
#include "chrono_vehicle/ChVehicle.h"

#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChSteeringController.h"
#include "chrono_vehicle/utils/ChAdaptiveSpeedController.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"

//to import/wrap
//#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/utils/ChUtilsChaseCamera.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"

/*using namespace chrono::irrlicht;
using namespace irr;*/
%}


#define ChApiIrr 


%import(module = "pychrono.core") "../chrono/core/ChBezierCurve.h"
%import(module = "pychrono.irrlicht") "dimension2d.h"
%import(module = "pychrono.irrlicht") "ChIrrAppInterface.i"
%import(module = "pychrono.irrlicht") "ChIrrApp.i"

//%shared_ptr(chrono::vehicle::RigidTerrain::Patch)

%template(vector_Entry) std::vector< chrono::vehicle::ChDataDriver::Entry >;


/* Parse the header file to generate wrappers */


%include "../chrono_vehicle/ChDriver.h"
%include "../chrono_vehicle/driver/ChDataDriver.h"
%include "../chrono_vehicle/utils/ChSpeedController.h"
%include "../chrono_vehicle/utils/ChSteeringController.h"
%include "../chrono_vehicle/utils/ChAdaptiveSpeedController.h"

%include "../chrono_vehicle/driver/ChPathFollowerDriver.h"
%include "../chrono_vehicle/driver/ChPathFollowerACCDriver.h"


%include "../chrono/utils/ChUtilsChaseCamera.h"
%include "../chrono_vehicle/utils/ChVehicleIrrApp.h"
%include "../chrono_vehicle/utils/ChVehicleIrrApp.h"
%include "../chrono_vehicle/driver/ChIrrGuiDriver.h"
