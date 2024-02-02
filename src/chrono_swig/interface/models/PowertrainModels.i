%{

/* Includes additional C++ in the wrapper code */

#include <string>
#include <vector>

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono/core/ChCubicSpline.h"

//#include "chrono_models/ChApiModels.h"

#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_EngineShafts.h"
#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_EngineSimpleMap.h"
#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_EngineSimple.h"
#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.h"
#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_AutomaticTransmissionSimpleMap.h"

#include "chrono_models/vehicle/sedan/Sedan_EngineSimpleMap.h"
#include "chrono_models/vehicle/sedan/Sedan_AutomaticTransmissionSimpleMap.h"

#include "chrono_models/vehicle/citybus/CityBus_EngineSimpleMap.h"
#include "chrono_models/vehicle/citybus/CityBus_AutomaticTransmissionSimpleMap.h"

#include "chrono_models/vehicle/man/powertrain/MAN_5t_EngineSimple.h"
#include "chrono_models/vehicle/man/powertrain/MAN_5t_AutomaticTransmissionSimple.h"
#include "chrono_models/vehicle/man/powertrain/MAN_5t_EngineSimpleMap.h"
#include "chrono_models/vehicle/man/powertrain/MAN_5t_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/man/powertrain/MAN_7t_EngineSimple.h"
#include "chrono_models/vehicle/man/powertrain/MAN_7t_AutomaticTransmissionSimple.h"
#include "chrono_models/vehicle/man/powertrain/MAN_7t_EngineSimpleMap.h"
#include "chrono_models/vehicle/man/powertrain/MAN_7t_AutomaticTransmissionSimpleMap.h"

#include "chrono_models/vehicle/uaz/UAZBUS_EngineSimpleMap.h"
#include "chrono_models/vehicle/uaz/UAZBUS_AutomaticTransmissionSimpleMap.h"

#include "chrono_models/vehicle/gator/Gator_EngineSimpleMap.h"
#include "chrono_models/vehicle/gator/Gator_EngineSimple.h"
#include "chrono_models/vehicle/gator/Gator_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/gator/Gator_AutomaticTransmissionSimple.h"

#include "chrono_models/vehicle/artcar/ARTcar_EngineSimpleMap.h"
#include "chrono_models/vehicle/artcar/ARTcar_AutomaticTransmissionSimpleMap.h"

#include "chrono_models/vehicle/feda/FEDA_EngineSimpleMap.h"
#include "chrono_models/vehicle/feda/FEDA_AutomaticTransmissionSimpleMap.h"

#include "chrono_models/vehicle/m113/powertrain/M113_AutomaticTransmissionShafts.h"
#include "chrono_models/vehicle/m113/powertrain/M113_AutomaticTransmissionSimple.h"
#include "chrono_models/vehicle/m113/powertrain/M113_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/m113/powertrain/M113_EngineShafts.h"
#include "chrono_models/vehicle/m113/powertrain/M113_EngineSimple.h"
#include "chrono_models/vehicle/m113/powertrain/M113_EngineSimpleMap.h"
%}

%shared_ptr(chrono::vehicle::hmmwv::HMMWV_EngineShafts)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_EngineSimpleMap)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_EngineSimple)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_AutomaticTransmissionShafts)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_AutomaticTransmissionSimpleMap)

%shared_ptr(chrono::vehicle::sedan::Sedan_EngineSimpleMap)
%shared_ptr(chrono::vehicle::sedan::Sedan_AutomaticTransmissionSimpleMap)

%shared_ptr(chrono::vehicle::citybus::CityBus_EngineSimpleMap)
%shared_ptr(chrono::vehicle::citybus::CityBus_AutomaticTransmissionSimpleMap)

%shared_ptr(chrono::vehicle::man::MAN_5t_EngineSimple)
%shared_ptr(chrono::vehicle::man::MAN_5t_AutomaticTransmissionSimple)
%shared_ptr(chrono::vehicle::man::MAN_5t_EngineSimpleMap)
%shared_ptr(chrono::vehicle::man::MAN_5t_AutomaticTransmissionSimpleMap)
%shared_ptr(chrono::vehicle::man::MAN_7t_EngineSimple)
%shared_ptr(chrono::vehicle::man::MAN_7t_AutomaticTransmissionSimple)
%shared_ptr(chrono::vehicle::man::MAN_7t_EngineSimpleMap)
%shared_ptr(chrono::vehicle::man::MAN_7t_AutomaticTransmissionSimpleMap)

%shared_ptr(chrono::vehicle::uaz::UAZBUS_EngineSimpleMap)
%shared_ptr(chrono::vehicle::uaz::UAZBUS_AutomaticTransmissionSimpleMap)

%shared_ptr(chrono::vehicle::artcar::ARTcar_EngineSimpleMap)
%shared_ptr(chrono::vehicle::artcar::ARTcar_AutomaticTransmissionSimpleMap)

%shared_ptr(chrono::vehicle::gator::Gator_EngineSimpleMap)
%shared_ptr(chrono::vehicle::gator::Gator_EngineSimple)
%shared_ptr(chrono::vehicle::gator::Gator_AutomaticTransmissionSimpleMap)
%shared_ptr(chrono::vehicle::gator::Gator_AutomaticTransmissionSimple)

%shared_ptr(chrono::vehicle::feda::FEDA_EngineSimpleMap)
%shared_ptr(chrono::vehicle::feda::FEDA_AutomaticTransmissionSimpleMap)

%shared_ptr(chrono::vehicle::m113::M113_AutomaticTransmissionShafts)
%shared_ptr(chrono::vehicle::m113::M113_AutomaticTransmissionSimple)
%shared_ptr(chrono::vehicle::m113::M113_AutomaticTransmissionSimpleMap)
%shared_ptr(chrono::vehicle::m113::M113_EngineShafts)
%shared_ptr(chrono::vehicle::m113::M113_EngineSimple)
%shared_ptr(chrono::vehicle::m113::M113_EngineSimpleMap)

%import "chrono_swig/interface/vehicle/ChPowertrain.i"

// Model:

%include "../../../chrono_models/vehicle/hmmwv/powertrain/HMMWV_EngineShafts.h"
%include "../../../chrono_models/vehicle/hmmwv/powertrain/HMMWV_EngineSimpleMap.h"
%include "../../../chrono_models/vehicle/hmmwv/powertrain/HMMWV_EngineSimple.h"
%include "../../../chrono_models/vehicle/hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.h"
%include "../../../chrono_models/vehicle/hmmwv/powertrain/HMMWV_AutomaticTransmissionSimpleMap.h"

%include "../../../chrono_models/vehicle/sedan/Sedan_EngineSimpleMap.h"
%include "../../../chrono_models/vehicle/sedan/Sedan_AutomaticTransmissionSimpleMap.h"

%include "../../../chrono_models/vehicle/citybus/CityBus_EngineSimpleMap.h"
%include "../../../chrono_models/vehicle/citybus/CityBus_AutomaticTransmissionSimpleMap.h"

%include "../../../chrono_models/vehicle/man/powertrain/MAN_5t_EngineSimple.h"
%include "../../../chrono_models/vehicle/man/powertrain/MAN_5t_AutomaticTransmissionSimple.h"
%include "../../../chrono_models/vehicle/man/powertrain/MAN_5t_EngineSimpleMap.h"
%include "../../../chrono_models/vehicle/man/powertrain/MAN_5t_AutomaticTransmissionSimpleMap.h"
%include "../../../chrono_models/vehicle/man/powertrain/MAN_7t_EngineSimple.h"
%include "../../../chrono_models/vehicle/man/powertrain/MAN_7t_AutomaticTransmissionSimple.h"
%include "../../../chrono_models/vehicle/man/powertrain/MAN_7t_EngineSimpleMap.h"
%include "../../../chrono_models/vehicle/man/powertrain/MAN_7t_AutomaticTransmissionSimpleMap.h"

%include "../../../chrono_models/vehicle/artcar/ARTcar_EngineSimpleMap.h"
%include "../../../chrono_models/vehicle/artcar/ARTcar_AutomaticTransmissionSimpleMap.h"

%include "../../../chrono_models/vehicle/uaz/UAZBUS_EngineSimpleMap.h"
%include "../../../chrono_models/vehicle/uaz/UAZBUS_AutomaticTransmissionSimpleMap.h"

%include "../../../chrono_models/vehicle/gator/Gator_EngineSimpleMap.h"
%include "../../../chrono_models/vehicle/gator/Gator_EngineSimple.h"
%include "../../../chrono_models/vehicle/gator/Gator_AutomaticTransmissionSimpleMap.h"
%include "../../../chrono_models/vehicle/gator/Gator_AutomaticTransmissionSimple.h"

%include "../../../chrono_models/vehicle/feda/FEDA_EngineSimpleMap.h"
%include "../../../chrono_models/vehicle/feda/FEDA_AutomaticTransmissionSimpleMap.h"

%include "../../../chrono_models/vehicle/m113/powertrain/M113_AutomaticTransmissionShafts.h"
%include "../../../chrono_models/vehicle/m113/powertrain/M113_AutomaticTransmissionSimple.h"
%include "../../../chrono_models/vehicle/m113/powertrain/M113_AutomaticTransmissionSimpleMap.h"
%include "../../../chrono_models/vehicle/m113/powertrain/M113_EngineShafts.h"
%include "../../../chrono_models/vehicle/m113/powertrain/M113_EngineSimple.h"
%include "../../../chrono_models/vehicle/m113/powertrain/M113_EngineSimpleMap.h"
