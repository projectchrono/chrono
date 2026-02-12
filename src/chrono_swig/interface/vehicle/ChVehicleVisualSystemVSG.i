%{
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::vsg3d;
using namespace chrono::vehicle;

%}

#ifdef SWIGCSHARP
    // Import ChVisualSystemVSG from vsg module (same pattern as irrlicht)
    // for C# to understand the multiple inheritance chain
    %import "../../../chrono_swig/interface/vsg/ChVisualSystemVSG.i"
    
    // Declare shared pointer for base VSG class so that swig handles dual inheritance
    %shared_ptr(chrono::vsg3d::ChVisualSystemVSG)
#endif

%shared_ptr(chrono::vehicle::ChVehicleVisualSystemVSG)
%shared_ptr(chrono::vehicle::ChWheeledVehicleVisualSystemVSG)
%shared_ptr(chrono::vehicle::ChTrackedVehicleVisualSystemVSG)
%shared_ptr(chrono::vehicle::ChSuspensionTestRigVisualSystemVSG)

%include "../../../chrono_vehicle/visualization/ChVehicleVisualSystemVSG.h"
%include "../../../chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
%include "../../../chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h"
%include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemVSG.h"

%DefSharedPtrDynamicCast(chrono::vehicle, ChVehicleVisualSystem, ChTrackedVehicleVisualSystemVSG)
%DefSharedPtrDynamicCast(chrono::vehicle, ChVehicleVisualSystem, ChWheeledVehicleVisualSystemVSG)
// Cross-namespace dynamic casts (for the multiple inheritance from vsg3d::ChVisualSystemVSG)
%DefSharedPtrDynamicCast2NS(chrono::vsg3d, chrono::vehicle, ChVisualSystemVSG, ChWheeledVehicleVisualSystemVSG)
%DefSharedPtrDynamicCast2NS(chrono::vsg3d, chrono::vehicle, ChVisualSystemVSG, ChTrackedVehicleVisualSystemVSG)
%DefSharedPtrDynamicCast2NS(chrono::vsg3d, chrono::vehicle, ChVisualSystemVSG, ChSuspensionTestRigVisualSystemVSG)
