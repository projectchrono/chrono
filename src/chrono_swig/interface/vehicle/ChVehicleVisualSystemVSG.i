%{
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::vsg3d;
using namespace chrono::vehicle;

%}

%shared_ptr(chrono::vehicle::ChVehicleVisualSystemVSG)
%shared_ptr(chrono::vehicle::ChWheeledVehicleVisualSystemVSG)

%include "../../../chrono_vehicle/visualization/ChVehicleVisualSystemVSG.h"
%include "../../../chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
