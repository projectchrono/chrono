%{

/* Includes additional C++ in the wrapper code */

#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_models/ChApiModels.h"

#include "chrono_models/vehicle/hmmwv/tire/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_Pac02Tire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_FialaTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_TMeasyTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_MBTire.h"
//#include "chrono_models/vehicle/hmmwv/tire/HMMWV_ReissnerTire.h"

#include "chrono_models/vehicle/sedan/Sedan_TMeasyTire.h"
#include "chrono_models/vehicle/sedan/Sedan_RigidTire.h"

#include "chrono_models/vehicle/citybus/CityBus_RigidTire.h"
#include "chrono_models/vehicle/citybus/CityBus_TMeasyTire.h"

#include "chrono_models/vehicle/man/MAN_5t_TMeasyTire.h"

#include "chrono_models/vehicle/uaz/UAZBUS_TMeasyTire.h"
#include "chrono_models/vehicle/uaz/UAZBUS_RigidTire.h"
#include "chrono_models/vehicle/uaz/UAZBUS_Pac02Tire.h"

#include "chrono_models/vehicle/gator/Gator_RigidTire.h"
#include "chrono_models/vehicle/gator/Gator_TMeasyTire.h"

#include "chrono_models/vehicle/artcar/ARTcar_RigidTire.h"

#include "chrono_models/vehicle/feda/FEDA_Pac02Tire.h"
#include "chrono_models/vehicle/feda/FEDA_RigidTire.h"

#include "chrono_models/vehicle/bmw/BMW_E90_TMsimpleTireFront.h"
#include "chrono_models/vehicle/bmw/BMW_E90_TMsimpleTireRear.h"
%}

%import "chrono_swig/interface/vehicle/ChTire.i"

%shared_ptr(chrono::vehicle::hmmwv::HMMWV_RigidTire)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Pac89Tire)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Pac02Tire)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_FialaTire)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_TMeasyTire)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_MBTire)
//%shared_ptr(chrono::vehicle::hmmwv::HMMWV_ReissnerTire)

%shared_ptr(chrono::vehicle::sedan::Sedan_TMeasyTire)
%shared_ptr(chrono::vehicle::sedan::Sedan_RigidTire)

%shared_ptr(chrono::vehicle::citybus::CityBus_RigidTire)
%shared_ptr(chrono::vehicle::citybus::CityBus_TMeasyTire)

%shared_ptr(chrono::vehicle::man::MAN_5t_TMeasyTire)

%shared_ptr(chrono::vehicle::uaz::UAZBUS_TMeasyTireFront)
%shared_ptr(chrono::vehicle::uaz::UAZBUS_TMeasyTireRear)
%shared_ptr(chrono::vehicle::uaz::UAZBUS_RigidTire)
%shared_ptr(chrono::vehicle::uaz::UAZBUS_Pac02Tire)

%shared_ptr(chrono::vehicle::gator::Gator_RigidTire_Front)
%shared_ptr(chrono::vehicle::gator::Gator_RigidTire_Rear)
%shared_ptr(chrono::vehicle::gator::Gator_TMeasyTire_Front)
%shared_ptr(chrono::vehicle::gator::Gator_TMeasyTire_Rear)

%shared_ptr(chrono::vehicle::artcar::ARTcar_RigidTire)

%shared_ptr(chrono::vehicle::feda::FEDA_Pac02Tire)
%shared_ptr(chrono::vehicle::feda::FEDA_RigidTire)

%shared_ptr(chrono::vehicle::bmw::BMW_E90_TMsimpleTireFront)
%shared_ptr(chrono::vehicle::bmw::BMW_E90_TMsimpleTireRear)

// Model:

%include "../../../chrono_models/vehicle/hmmwv/tire/HMMWV_RigidTire.h"
%include "../../../chrono_models/vehicle/hmmwv/tire/HMMWV_Pac89Tire.h"
%include "../../../chrono_models/vehicle/hmmwv/tire/HMMWV_Pac02Tire.h"
%include "../../../chrono_models/vehicle/hmmwv/tire/HMMWV_FialaTire.h"
%include "../../../chrono_models/vehicle/hmmwv/tire/HMMWV_TMeasyTire.h"
%include "../../../chrono_models/vehicle/hmmwv/tire/HMMWV_MBTire.h"
//%include "../../../chrono_models/vehicle/hmmwv/tire/HMMWV_ReissnerTire.h"

%include "../../../chrono_models/vehicle/sedan/Sedan_TMeasyTire.h"
%include "../../../chrono_models/vehicle/sedan/Sedan_RigidTire.h"

%include "../../../chrono_models/vehicle/citybus/CityBus_RigidTire.h"
%include "../../../chrono_models/vehicle/citybus/CityBus_TMeasyTire.h"

%include "../../../chrono_models/vehicle/man/MAN_5t_TMeasyTire.h"

%include "../../../chrono_models/vehicle/uaz/UAZBUS_TMeasyTire.h"
%include "../../../chrono_models/vehicle/uaz/UAZBUS_RigidTire.h"
%include "../../../chrono_models/vehicle/uaz/UAZBUS_Pac02Tire.h"

%include "../../../chrono_models/vehicle/gator/Gator_RigidTire.h"
%include "../../../chrono_models/vehicle/gator/Gator_TMeasyTire.h"

%include "../../../chrono_models/vehicle/artcar/ARTcar_RigidTire.h"

%include "../../../chrono_models/vehicle/feda/FEDA_Pac02Tire.h"
%include "../../../chrono_models/vehicle/feda/FEDA_RigidTire.h"

%include "../../../chrono_models/vehicle/bmw/BMW_E90_TMsimpleTireFront.h"
%include "../../../chrono_models/vehicle/bmw/BMW_E90_TMsimpleTireRear.h"
