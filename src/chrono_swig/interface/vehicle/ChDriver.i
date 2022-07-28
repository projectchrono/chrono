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
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDataDriverSTR.h"

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/utils/ChVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChIrrGuiDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleVisualSystemIrrlicht.h"

//using namespace chrono::irrlicht;
//using namespace irr;
#endif

#endif             // --------------------------------------------------------------------- PYTHON

%}

#ifdef SWIGCSHARP
%import "../chrono/core/ChBezierCurve.h"
#endif

#ifdef SWIGPYCHRONO
%import(module = "pychrono.core") "../chrono/core/ChBezierCurve.h"
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
%shared_ptr(chrono::vehicle::ChClosedLoopDriver)
%shared_ptr(chrono::vehicle::ChPathFollowerDriver)
%shared_ptr(chrono::vehicle::ChPathFollowerDriverXT)
%shared_ptr(chrono::vehicle::ChPathFollowerDriverSR)
%shared_ptr(chrono::vehicle::ChPathFollowerDriverStanley)
%shared_ptr(chrono::vehicle::ChPathFollowerACCDriver)
%shared_ptr(chrono::vehicle::ChDriverSTR)
%shared_ptr(chrono::vehicle::ChDataDriverSTR)

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON
#ifdef CHRONO_IRRLICHT
%shared_ptr(chrono::vehicle::ChIrrGuiDriver)
%shared_ptr(chrono::vehicle::ChIrrGuiDriverSTR)
#endif
#endif             // --------------------------------------------------------------------- PYTHON


%rename(DataDriverEntry) chrono::vehicle::ChDataDriver::Entry;
%template(vector_Entry) std::vector< chrono::vehicle::ChDataDriver::Entry >;

// Parse the header file to generate wrappers
%include "../../../chrono/utils/ChUtilsChaseCamera.h"
%include "../../../chrono_vehicle/ChDriver.h"
%include "../../../chrono_vehicle/driver/ChDataDriver.h"
%include "../../../chrono_vehicle/driver/ChPathFollowerDriver.h"
%include "../../../chrono_vehicle/driver/ChPathFollowerACCDriver.h"
%include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChDriverSTR.h"
%include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChDataDriverSTR.h"
%include "../../../chrono_vehicle/utils/ChSpeedController.h"
%include "../../../chrono_vehicle/utils/ChSteeringController.h"
%include "../../../chrono_vehicle/utils/ChAdaptiveSpeedController.h"

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

#ifdef CHRONO_IRRLICHT
%include "../../../chrono_vehicle/utils/ChVehicleVisualSystemIrrlicht.h"
%include "../../../chrono_vehicle/driver/ChIrrGuiDriver.h"
%include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChIrrGuiDriverSTR.h"
%include "../../../chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"
%include "../../../chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleVisualSystemIrrlicht.h"
%include "irrTypes.h"
%include "vector2d.h"
%include "dimension2d.h"
%template(dimension2du) irr::core::dimension2d<irr::u32>;
#endif

#endif             // --------------------------------------------------------------------- PYTHON
