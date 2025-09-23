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
%DefSharedPtrDynamicCast2NS(chrono::vsg3d, chrono::vehicle, ChVisualSystemVSG, ChSuspensionTestRigVisualSystemVSG)
