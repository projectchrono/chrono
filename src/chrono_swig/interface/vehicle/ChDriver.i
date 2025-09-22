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
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigInteractiveDriver.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/visualization/ChVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
#endif

#include "chrono_vehicle/driver/ChHumanDriver.h"
%}

#ifdef SWIGPYCHRONO
%import(module = "pychrono.core") "../../../chrono/core/ChBezierCurve.h"
#endif

%import "../../../chrono/core/ChBezierCurve.h"
%import "ChPowertrain.i"
%import "ChChassis.i"

%shared_ptr(chrono::vehicle::ChDriver)
%shared_ptr(chrono::vehicle::ChDataDriver)
%shared_ptr(chrono::vehicle::ChHumanDriver)
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
%shared_ptr(chrono::vehicle::ChSuspensionTestRigInteractiveDriver)

%rename(DriverInputs) chrono::vehicle::ChDriver::Inputs;
%rename(DataDriverEntry) Entry;
%template(vector_Entry) std::vector< chrono::vehicle::ChDataDriver::Entry >;

// Parse the header file to generate wrappers
%include "../../../chrono/utils/ChUtilsChaseCamera.h"
%include "../../../chrono_vehicle/ChDriver.h"
%include "../../../chrono_vehicle/driver/ChDataDriver.h"
%include "../../../chrono_vehicle/driver/ChInteractiveDriver.h"
%include "../../../chrono_vehicle/driver/ChPathFollowerDriver.h"
%include "../../../chrono_vehicle/driver/ChPathFollowerACCDriver.h"
%include "../../../chrono_vehicle/driver/ChHumanDriver.h"
%include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDriver.h"
%include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDataDriver.h"
%include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigInteractiveDriver.h"
%include "../../../chrono_vehicle/utils/ChSpeedController.h"
%include "../../../chrono_vehicle/utils/ChSteeringController.h"
%include "../../../chrono_vehicle/utils/ChAdaptiveSpeedController.h"


