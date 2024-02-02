%{

/* Includes additional C++ in the wrapper code */

#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_models/ChApiModels.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"
#include "chrono_models/vehicle/sedan/Sedan_Wheel.h"
#include "chrono_models/vehicle/citybus/CityBus_Wheel.h"
#include "chrono_models/vehicle/man/MAN_5t_Wheel.h"
#include "chrono_models/vehicle/uaz/UAZBUS_Wheel.h"
#include "chrono_models/vehicle/gator/Gator_Wheel.h"
#include "chrono_models/vehicle/artcar/ARTcar_Wheel.h"
#include "chrono_models/vehicle/feda/FEDA_Wheel.h"
%}


%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Wheel)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_WheelLeft)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_WheelRight)

%shared_ptr(chrono::vehicle::sedan::Sedan_Wheel)
%shared_ptr(chrono::vehicle::sedan::Sedan_WheelLeft)
%shared_ptr(chrono::vehicle::sedan::Sedan_WheelRight)

%shared_ptr(chrono::vehicle::citybus::CityBus_Wheel)
%shared_ptr(chrono::vehicle::citybus::CityBus_WheelLeft)
%shared_ptr(chrono::vehicle::citybus::CityBus_WheelRight)

%shared_ptr(chrono::vehicle::man::MAN_5t_Wheel)
%shared_ptr(chrono::vehicle::man::MAN_5t_WheelLeft)
%shared_ptr(chrono::vehicle::man::MAN_5t_WheelRight)

%shared_ptr(chrono::vehicle::uaz::UAZBUS_Wheel)

%shared_ptr(chrono::vehicle::gator::Gator_Wheel)

%shared_ptr(chrono::vehicle::artcar::ARTcar_Wheel)
%shared_ptr(chrono::vehicle::artcar::ARTcar_WheelLeft)
%shared_ptr(chrono::vehicle::artcar::ARTcar_WheelRight)

%shared_ptr(chrono::vehicle::feda::FEDA_Wheel)

%import "../../../chrono_vehicle/wheeled_vehicle/ChWheel.h"

// Model:

%include "../../../chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"
%include "../../../chrono_models/vehicle/sedan/Sedan_Wheel.h"
%include "../../../chrono_models/vehicle/citybus/CityBus_Wheel.h"
%include "../../../chrono_models/vehicle/man/MAN_5t_Wheel.h"
%include "../../../chrono_models/vehicle/uaz/UAZBUS_Wheel.h"
%include "../../../chrono_models/vehicle/gator/Gator_Wheel.h"
%include "../../../chrono_models/vehicle/artcar/ARTcar_Wheel.h"
%include "../../../chrono_models/vehicle/feda/FEDA_Wheel.h"
