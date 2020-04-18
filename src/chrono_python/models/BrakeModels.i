%{

/* Includes additional C++ in the wrapper code */


#include "chrono_vehicle/ChVehicleModelData.h"


#include "chrono_models/ChApiModels.h"

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/vehicle/generic/Generic_BrakeSimple.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_BrakeSimple.h"
#include "chrono_models/vehicle/sedan/Sedan_BrakeSimple.h"
#include "chrono_models/vehicle/citybus/CityBus_BrakeSimple.h"
#include "chrono_models/vehicle/man/MAN_5t_BrakeSimple.h"
#include "chrono_models/vehicle/uaz/UAZBUS_BrakeSimple.h"
%}



%import "../chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

%shared_ptr(chrono::vehicle::generic::Generic_BrakeSimple)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_BrakeSimple)
%shared_ptr(chrono::vehicle::sedan::Sedan_BrakeSimple)
%shared_ptr(chrono::vehicle::citybus::CityBus_BrakeSimple)
%shared_ptr(chrono::vehicle::man::MAN_5t_BrakeSimple)
%shared_ptr(chrono::vehicle::uaz::UAZBUS_BrakeSimpleFront)
%shared_ptr(chrono::vehicle::uaz::UAZBUS_BrakeSimpleRear)

// Model:

%include "../../chrono_models/vehicle/generic/Generic_BrakeSimple.h"
%include "../../chrono_models/vehicle/hmmwv/HMMWV_BrakeSimple.h"
%include "../../chrono_models/vehicle/sedan/Sedan_BrakeSimple.h"
%include "../../chrono_models/vehicle/citybus/CityBus_BrakeSimple.h"
%include "../../chrono_models/vehicle/man/MAN_5t_BrakeSimple.h"
%include "../../chrono_models/vehicle/uaz/UAZBUS_BrakeSimple.h"
