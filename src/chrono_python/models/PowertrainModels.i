%{

/* Includes additional C++ in the wrapper code */

#include <string>
#include <vector>

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChPowertrain.h"
#include "chrono/core/ChCubicSpline.h"

//#include "chrono_models/ChApiModels.h"

#include "chrono_models/vehicle/generic/Generic_SimplePowertrain.h"
#include "chrono_models/vehicle/generic/Generic_SimpleMapPowertrain.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_SimplePowertrain.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_SimpleMapPowertrain.h"

#include "chrono_models/vehicle/sedan/Sedan_SimpleMapPowertrain.h"

#include "chrono_models/vehicle/citybus/CityBus_SimpleMapPowertrain.h"

#include "chrono_models/vehicle/man/MAN_5t_SimpleMapPowertrain.h"
#include "chrono_models/vehicle/man/MAN_5t_SimpleCVTPowertrain.h"
#include "chrono_models/vehicle/man/MAN_7t_SimpleMapPowertrain.h"
#include "chrono_models/vehicle/man/MAN_7t_SimpleCVTPowertrain.h"

#include "chrono_models/vehicle/uaz/UAZBUS_SimpleMapPowertrain.h"

#include "chrono_models/vehicle/gator/Gator_SimplePowertrain.h"
#include "chrono_models/vehicle/gator/Gator_SimpleMapPowertrain.h"

#include "chrono_models/vehicle/m113/M113_SimplePowertrain.h"
#include "chrono_models/vehicle/m113/M113_ShaftsPowertrain.h"
%}


%shared_ptr(chrono::vehicle::generic::Generic_SimplePowertrain)
%shared_ptr(chrono::vehicle::generic::Generic_SimpleMapPowertrain)

%shared_ptr(chrono::vehicle::hmmwv::HMMWV_SimplePowertrain)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_SimpleMapPowertrain)

%shared_ptr(chrono::vehicle::sedan::Sedan_SimpleMapPowertrain)

%shared_ptr(chrono::vehicle::citybus::CityBus_SimpleMapPowertrain)

%shared_ptr(chrono::vehicle::man::MAN_5t_SimpleMapPowertrain)
%shared_ptr(chrono::vehicle::man::MAN_5t_SimpleCVTPowertrain)
%shared_ptr(chrono::vehicle::man::MAN_7t_SimpleMapPowertrain)
%shared_ptr(chrono::vehicle::man::MAN_7t_SimpleCVTPowertrain)

%shared_ptr(chrono::vehicle::uaz::UAZBUS_SimpleMapPowertrain)

%shared_ptr(chrono::vehicle::gator::Gator_SimplePowertrain)
%shared_ptr(chrono::vehicle::gator::Gator_SimpleMapPowertrain)

%shared_ptr(chrono::vehicle::m113::M113_SimplePowertrain)
%shared_ptr(chrono::vehicle::m113::M113_ShaftsPowertrain)

%import "chrono_python/vehicle/ChPowertrain.i"

// Model:
%include "../../chrono_models/vehicle/generic/Generic_SimplePowertrain.h"
%include "../../chrono_models/vehicle/generic/Generic_SimpleMapPowertrain.h"

%include "../../chrono_models/vehicle/hmmwv/HMMWV_SimplePowertrain.h"
%include "../../chrono_models/vehicle/hmmwv/HMMWV_SimpleMapPowertrain.h"

%include "../../chrono_models/vehicle/sedan/Sedan_SimpleMapPowertrain.h"

%include "../../chrono_models/vehicle/citybus/CityBus_SimpleMapPowertrain.h"

%include "../../chrono_models/vehicle/man/MAN_5t_SimpleMapPowertrain.h"
%include "../../chrono_models/vehicle/man/MAN_5t_SimpleCVTPowertrain.h"
%include "../../chrono_models/vehicle/man/MAN_7t_SimpleMapPowertrain.h"
%include "../../chrono_models/vehicle/man/MAN_7t_SimpleCVTPowertrain.h"

%include "../../chrono_models/vehicle/uaz/UAZBUS_SimpleMapPowertrain.h"

%include "../../chrono_models/vehicle/gator/Gator_SimplePowertrain.h"
%include "../../chrono_models/vehicle/gator/Gator_SimpleMapPowertrain.h"

%include "../../chrono_models/vehicle/m113/M113_SimplePowertrain.h"
%include "../../chrono_models/vehicle/m113/M113_ShaftsPowertrain.h"
