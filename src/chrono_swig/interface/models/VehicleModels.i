%{

/* Includes additional C++ in the wrapper code */

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChContactMaterial.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

#include "chrono_models/ChApiModels.h"


#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Vehicle.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_VehicleReduced.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_VehicleFull.h"

#include "chrono_models/vehicle/sedan/Sedan.h"
#include "chrono_models/vehicle/sedan/Sedan_Vehicle.h"

#include "chrono_models/vehicle/citybus/CityBus.h"
#include "chrono_models/vehicle/citybus/CityBus_Vehicle.h"

#include "chrono_models/vehicle/man/MAN_5t.h"
#include "chrono_models/vehicle/man/MAN_7t.h"
#include "chrono_models/vehicle/man/MAN_10t.h"
#include "chrono_models/vehicle/man/MAN_5t_Vehicle.h"
#include "chrono_models/vehicle/man/MAN_7t_Vehicle.h"
#include "chrono_models/vehicle/man/MAN_10t_Vehicle.h"

#include "chrono_models/vehicle/uaz/UAZBUS.h"
#include "chrono_models/vehicle/uaz/UAZBUS_Vehicle.h"

#include "chrono_models/vehicle/gator/Gator.h"
#include "chrono_models/vehicle/gator/Gator_Vehicle.h"

#include "chrono_models/vehicle/artcar/ARTcar.h"
#include "chrono_models/vehicle/artcar/ARTcar_Vehicle.h"

#include "chrono_models/vehicle/feda/FEDA.h"
#include "chrono_models/vehicle/feda/FEDA_Vehicle.h"

#include "chrono_models/vehicle/bmw/BMW_E90.h"
#include "chrono_models/vehicle/bmw/BMW_E90_Vehicle.h"

#include "chrono_models/vehicle/m113/M113.h"
#include "chrono_models/vehicle/m113/M113_Vehicle.h"

#include "chrono_models/vehicle/kraz/Kraz.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor.h"
#include "chrono_models/vehicle/kraz/Kraz_trailer.h"
%}


%shared_ptr(chrono::vehicle::hmmwv::HMMWV)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Vehicle)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_VehicleReduced)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_VehicleFull)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Reduced)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Full)

%shared_ptr(chrono::vehicle::sedan::Sedan)
%shared_ptr(chrono::vehicle::sedan::Sedan_Vehicle)

%shared_ptr(chrono::vehicle::citybus::CityBus)
%shared_ptr(chrono::vehicle::citybus::CityBus_Vehicle)

%shared_ptr(chrono::vehicle::man::MAN_5t)
%shared_ptr(chrono::vehicle::man::MAN_7t)
%shared_ptr(chrono::vehicle::man::MAN_10t)
%shared_ptr(chrono::vehicle::man::MAN_5t_Vehicle)
%shared_ptr(chrono::vehicle::man::MAN_7t_Vehicle)
%shared_ptr(chrono::vehicle::man::MAN_10t_Vehicle)

%shared_ptr(chrono::vehicle::uaz::UAZBUS)
%shared_ptr(chrono::vehicle::uaz::UAZBUS_Vehicle)

%shared_ptr(chrono::vehicle::gator::Gator)
%shared_ptr(chrono::vehicle::gator::Gator_Vehicle)

%shared_ptr(chrono::vehicle::artcar::ARTcar)
%shared_ptr(chrono::vehicle::artcar::ARTcar_Vehicle)

%shared_ptr(chrono::vehicle::feda::FEDA)
%shared_ptr(chrono::vehicle::feda::FEDA_Vehicle)

%shared_ptr(chrono::vehicle::bmw::BMW_E90)
%shared_ptr(chrono::vehicle::bmw::BMW_E90_Vehicle)

%shared_ptr(chrono::vehicle::m113::M113)
%shared_ptr(chrono::vehicle::m113::M113_Vehicle)
%shared_ptr(chrono::vehicle::m113::M113_Vehicle_SinglePin)
%shared_ptr(chrono::vehicle::m113::M113_Vehicle_DoublePin)
%shared_ptr(chrono::vehicle::m113::M113_Vehicle_BandBushing)
%shared_ptr(chrono::vehicle::m113::M113_Vehicle_BandANCF)

%shared_ptr(chrono::vehicle::kraz::kraz)
%shared_ptr(chrono::vehicle::kraz::Kraz_tractor)
%shared_ptr(chrono::vehicle::kraz::Kraz_trailer)


#ifdef SWIGCSHARP   // --------------------------------------------------------------------- CSHARP
%import "chrono_swig/interface/core/ChContactMaterial.i"
#endif           // --------------------------------------------------------------------- CSHARP

#ifdef SWIGPYCHRONO
%import(module = "pychrono.core") "chrono_swig/interface/core/ChContactMaterial.i"
#endif

// these are redundant - already imported in chmodulevehicle.i
// %import "chrono_swig/interface/vehicle/ChSuspension.i"
// %import "chrono_swig/interface/vehicle/ChDriveline.i"
// %import "chrono_swig/interface/vehicle/ChSteering.i"
// %import "chrono_swig/interface/vehicle/ChPowertrain.i"
// %import "chrono_swig/interface/vehicle/ChChassis.i"
// %import "chrono_swig/interface/vehicle/ChTire.i"
%import "chrono_swig/interface/vehicle/ChTrackAssembly.i"


#ifdef SWIGCSHARP   // --------------------------------------------------------------------- CSHARP

// These directives must come before the %import statements that process these headers - allows SWIG
// to understand that these methods override base class virtual functions (for Unity)
%csmethodmodifiers chrono::vehicle::ChWheeledVehicle::Synchronize "public override"
%csmethodmodifiers chrono::vehicle::ChTrackedVehicle::Synchronize "public override"

#endif              // --------------------------------------------------------------------- CSHARP

%import "../../../chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
%import "../../../chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
%import "../../../chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"
%import "../../../chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"
%import "../../../chrono_vehicle/ChVehicle.h"

// Model:

%include "../../../chrono_models/vehicle/hmmwv/HMMWV.h"
%include "../../../chrono_models/vehicle/hmmwv/HMMWV_Vehicle.h"
%include "../../../chrono_models/vehicle/hmmwv/HMMWV_VehicleReduced.h"
%include "../../../chrono_models/vehicle/hmmwv/HMMWV_VehicleFull.h"

%include "../../../chrono_models/vehicle/sedan/Sedan.h"
%include "../../../chrono_models/vehicle/sedan/Sedan_Vehicle.h"

%include "../../../chrono_models/vehicle/citybus/CityBus.h"
%include "../../../chrono_models/vehicle/citybus/CityBus_Vehicle.h"

%include "../../../chrono_models/vehicle/man/MAN_5t.h"
%include "../../../chrono_models/vehicle/man/MAN_7t.h"
%include "../../../chrono_models/vehicle/man/MAN_10t.h"
%include "../../../chrono_models/vehicle/man/MAN_5t_Vehicle.h"
%include "../../../chrono_models/vehicle/man/MAN_7t_Vehicle.h"
%include "../../../chrono_models/vehicle/man/MAN_10t_Vehicle.h"

%include "../../../chrono_models/vehicle/uaz/UAZBUS.h"
%include "../../../chrono_models/vehicle/uaz/UAZBUS_Vehicle.h"

%include "../../../chrono_models/vehicle/gator/Gator.h"
%include "../../../chrono_models/vehicle/gator/Gator_Vehicle.h"

%include "../../../chrono_models/vehicle/artcar/ARTcar.h"
%include "../../../chrono_models/vehicle/artcar/ARTcar_Vehicle.h"

%include "../../../chrono_models/vehicle/feda/FEDA.h"
%include "../../../chrono_models/vehicle/feda/FEDA_Vehicle.h"

%include "../../../chrono_models/vehicle/bmw/BMW_E90.h"
%include "../../../chrono_models/vehicle/bmw/BMW_E90_Vehicle.h"

%include "../../../chrono_models/vehicle/m113/M113.h"
%include "../../../chrono_models/vehicle/m113/M113_Vehicle.h"

%include "../../../chrono_models/vehicle/kraz/Kraz.h"
%include "../../../chrono_models/vehicle/kraz/Kraz_tractor.h"
%include "../../../chrono_models/vehicle/kraz/Kraz_trailer.h"
