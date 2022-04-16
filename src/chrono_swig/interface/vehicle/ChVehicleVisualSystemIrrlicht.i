%{
#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "chrono_vehicle/utils/ChVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
%}

%shared_ptr(chrono::vehicle::ChVehicleVisualSystem)
%shared_ptr(chrono::vehicle::ChVehicleVisualSystemIrrlicht)
%shared_ptr(chrono::vehicle::ChTrackedVehicleVisualSystemIrrlicht)
%shared_ptr(chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht)

%import(module = "pychrono.irrlicht") "chrono_swig/interface/irrlicht/ChVisualSystemIrrlicht.i"

%include "../../../chrono_vehicle/ChVehicleVisualSystem.h"
%include "../../../chrono_vehicle/utils/ChVehicleVisualSystemIrrlicht.h"
%include "../../../chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleVisualSystemIrrlicht.h"
%include "../../../chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

//%DefSharedPtrDynamicDowncast2NS(chrono::irrlicht, chrono::vehicle, ChVisualSystemIrrlicht, ChVehicleVisualSystemIrrlicht)
//%DefSharedPtrDynamicDowncast(chrono::vehicle, ChVehicleVisualSystem, ChVehicleVisualSystemIrrlicht)
%DefSharedPtrDynamicDowncast(chrono::vehicle, ChVehicleVisualSystem, ChTrackedVehicleVisualSystemIrrlicht)
%DefSharedPtrDynamicDowncast(chrono::vehicle, ChVehicleVisualSystem, ChWheeledVehicleVisualSystemIrrlicht)
//%DefSharedPtrDynamicDowncast(chrono::vehicle, ChVehicleVisualSystemIrrlicht, ChTrackedVehicleVisualSystemIrrlicht)
//%DefSharedPtrDynamicDowncast(chrono::vehicle, ChVehicleVisualSystemIrrlicht, ChWheeledVehicleVisualSystemIrrlicht)
