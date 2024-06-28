%{

/* Includes additional C++ in the wrapper code */
#include <string>
#include <vector>

#include "chrono/ChConfig.h"

#include "chrono/core/ChBezierCurve.h"
#include "chrono/utils/ChUtilsChaseCamera.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChSteeringController.h"
#include "chrono_vehicle/utils/ChAdaptiveSpeedController.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDriver.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDataDriver.h"

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/ChVehicleVisualSystemIrrlicht.h"// For the csharp module, these can be found in the ChVehicleVisualSystemIrrlicht
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"// For the csharp module, these can be found in the ChVehicleVisualSystemIrrlicht
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"

//using namespace chrono::irrlicht;
//using namespace irr;
#endif

#endif             // --------------------------------------------------------------------- PYTHON

#ifdef SWIGCSHARP //--------------------------------------------------------------------------CSHARP
#include "chrono_vehicle/driver/ChHumanDriver.h"
#endif            //--------------------------------------------------------------------------CSHARP
%}

#ifdef SWIGPYCHRONO
%import(module = "pychrono.core") "../../../chrono/core/ChBezierCurve.h"
#endif

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

#define ChApiIrr 

#ifdef CHRONO_IRRLICHT
%import(module = "pychrono.irrlicht") "dimension2d.h"
%import(module = "pychrono.irrlicht") "../irrlicht/ChVisualSystemIrrlicht.i"
#endif

#endif             // --------------------------------------------------------------------- PYTHON

%import "../../../chrono/core/ChBezierCurve.h"
%import "ChPowertrain.i"
%import "ChChassis.i"
%import "../../../chrono_vehicle/ChVehicle.h"

%shared_ptr(chrono::vehicle::ChDriver)
%shared_ptr(chrono::vehicle::ChDataDriver)
%shared_ptr(chrono::vehicle::ChInteractiveDriver)
%shared_ptr(chrono::vehicle::ChClosedLoopDriver)
%shared_ptr(chrono::vehicle::ChPathFollowerDriver)
%shared_ptr(chrono::vehicle::ChPathFollowerDriverXT)
%shared_ptr(chrono::vehicle::ChPathFollowerDriverSR)
%shared_ptr(chrono::vehicle::ChPathFollowerDriverPP)
%shared_ptr(chrono::vehicle::ChPathFollowerDriverStanley)
%shared_ptr(chrono::vehicle::ChPathFollowerACCDriver)
%shared_ptr(chrono::vehicle::ChSuspensionTestRigDriver)
%shared_ptr(chrono::vehicle::ChSuspensionTestRigDataDriver)

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON
#ifdef CHRONO_IRRLICHT
%shared_ptr(chrono::vehicle::ChInteractiveDriverIRR)
%shared_ptr(chrono::vehicle::ChSuspensionTestRigInteractiveDriverIRR)
#endif
#endif             // --------------------------------------------------------------------- PYTHON

#ifdef SWIGCSHARP //--------------------------------------------------------------------------CSHARP
%shared_ptr(chrono::vehicle::ChHumanDriver)
#endif            //--------------------------------------------------------------------------CSHARP



// Are these python specific?
%rename(DriverInputs) chrono::vehicle::ChDriver::Inputs;
%rename(DataDriverEntry) chrono::vehicle::ChDataDriver::Entry;
%template(vector_Entry) std::vector< chrono::vehicle::ChDataDriver::Entry >;

// Parse the header file to generate wrappers
%include "../../../chrono/utils/ChUtilsChaseCamera.h"
%include "../../../chrono_vehicle/ChDriver.h"
%include "../../../chrono_vehicle/driver/ChDataDriver.h"
%include "../../../chrono_vehicle/driver/ChInteractiveDriver.h"
%include "../../../chrono_vehicle/driver/ChPathFollowerDriver.h"
%include "../../../chrono_vehicle/driver/ChPathFollowerACCDriver.h"
%include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDriver.h"
%include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDataDriver.h"
%include "../../../chrono_vehicle/utils/ChSpeedController.h"
%include "../../../chrono_vehicle/utils/ChSteeringController.h"
%include "../../../chrono_vehicle/utils/ChAdaptiveSpeedController.h"



#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

#ifdef CHRONO_IRRLICHT
%include "../../../chrono_vehicle/ChVehicleVisualSystemIrrlicht.h"
%include "../../../chrono_vehicle/driver/ChInteractiveDriverIRR.h"
%include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigInteractiveDriverIRR.h"
%include "../../../chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
%include "../../../chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
%include "irrTypes.h"
%include "vector2d.h"
%include "dimension2d.h"
%template(dimension2du) irr::core::dimension2d<irr::u32>;

%DefSharedPtrDynamicCast(chrono::vehicle,ChInteractiveDriver, ChInteractiveDriverIRR)

#endif

#endif             // --------------------------------------------------------------------- PYTHON

#ifdef SWIGCSHARP //--------------------------------------------------------------------------CSHARP
%include "../../../chrono_vehicle/driver/ChHumanDriver.h"
#endif            //--------------------------------------------------------------------------CSHARP
