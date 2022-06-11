%{

/* Includes additional C++ in the wrapper code */

#include <string>
#include <vector>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChFrame.h"
#include "chrono/assets/ChColor.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono/physics/ChShaft.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

#include "chrono_models/vehicle/generic/Generic_RigidSuspension.h"
#include "chrono_models/vehicle/generic/Generic_RigidPinnedAxle.h"
#include "chrono_models/vehicle/generic/Generic_MultiLink.h"
#include "chrono_models/vehicle/generic/Generic_DoubleWishbone.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_DoubleWishbone.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_DoubleWishboneReduced.h"

#include "chrono_models/vehicle/sedan/Sedan_DoubleWishbone.h"
#include "chrono_models/vehicle/sedan/Sedan_MultiLink.h"

#include "chrono_models/vehicle/citybus/CityBus_ToeBarLeafspringAxle.h"
#include "chrono_models/vehicle/citybus/CityBus_SolidAxle.h"
#include "chrono_models/vehicle/citybus/CityBus_LeafspringAxle.h"

#include "chrono_models/vehicle/man/MAN_5t_BellcrankSolid3LinkAxle.h"
#include "chrono_models/vehicle/man/MAN_5t_Solid3LinkAxle.h"
#include "chrono_models/vehicle/man/MAN_10t_Front1Axle.h"
#include "chrono_models/vehicle/man/MAN_10t_Front2Axle.h"

#include "chrono_models/vehicle/uaz/UAZBUS_ToeBarLeafspringAxle.h"
#include "chrono_models/vehicle/uaz/UAZBUS_LeafspringAxle.h"

#include "chrono_models/vehicle/gator/Gator_SingleWishbone.h"
#include "chrono_models/vehicle/gator/Gator_RigidSuspension.h"

#include "chrono_models/vehicle/rccar/RCCar_DoubleWishbone.h"

#include "chrono_models/vehicle/feda/FEDA_DoubleWishbone.h"
%}


%shared_ptr(chrono::vehicle::generic::Generic_RigidSuspension)
%shared_ptr(chrono::vehicle::generic::Generic_RigidPinnedAxle)
%shared_ptr(chrono::vehicle::generic::Generic_MultiLink)
%shared_ptr(chrono::vehicle::generic::Generic_DoubleWishbone)
%shared_ptr(chrono::vehicle::generic::Generic_DoubleWishboneFront)
%shared_ptr(chrono::vehicle::generic::Generic_DoubleWishboneRear)

%shared_ptr(chrono::vehicle::hmmwv::HMMWV_DoubleWishbone)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_DoubleWishboneReduced)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_DoubleWishboneRear)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_DoubleWishboneReducedRear)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_DoubleWishboneFront)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_DoubleWishboneReducedFront)

%shared_ptr(chrono::vehicle::sedan::Sedan_DoubleWishbone)
%shared_ptr(chrono::vehicle::sedan::Sedan_MultiLink)

%shared_ptr(chrono::vehicle::citybus::CityBus_ToeBarLeafspringAxle)
%shared_ptr(chrono::vehicle::citybus::CityBus_SolidAxleFront)
%shared_ptr(chrono::vehicle::citybus::CityBus_SolidAxleRear)
%shared_ptr(chrono::vehicle::citybus::CityBus_LeafspringAxle)

%shared_ptr(chrono::vehicle::man::MAN_5t_BellcrankSolid3LinkAxle)
%shared_ptr(chrono::vehicle::man::MAN_5t_Solid3LinkAxle)
%shared_ptr(chrono::vehicle::man::MAN_10t_Front1Axle)
%shared_ptr(chrono::vehicle::man::MAN_10t_Front2Axle)

%shared_ptr(chrono::vehicle::uaz::UAZBUS_ToeBarLeafspringAxle)
%shared_ptr(chrono::vehicle::uaz::UAZBUS_LeafspringAxle)

%shared_ptr(chrono::vehicle::gator::Gator_SingleWishbone)
%shared_ptr(chrono::vehicle::gator::Gator_RigidSuspension)

%shared_ptr(chrono::vehicle::rccar::RCCar_DoubleWishbone)
%shared_ptr(chrono::vehicle::rccar::RCCar_DoubleWishboneRear)
%shared_ptr(chrono::vehicle::rccar::RCCar_DoubleWishboneFront)

%shared_ptr(chrono::vehicle::feda::FEDA_DoubleWishbone)
%shared_ptr(chrono::vehicle::feda::FEDA_DoubleWishboneRear)
%shared_ptr(chrono::vehicle::feda::FEDA_DoubleWishboneFront)

/* Parse the header file to generate wrappers */
%import "chrono_swig/interface/vehicle/ChSuspension.i"

// Model:
%include "../../../chrono_models/vehicle/generic/Generic_RigidSuspension.h"
%include "../../../chrono_models/vehicle/generic/Generic_RigidPinnedAxle.h"
%include "../../../chrono_models/vehicle/generic/Generic_MultiLink.h"
%include "../../../chrono_models/vehicle/generic/Generic_DoubleWishbone.h"

%include "../../../chrono_models/vehicle/hmmwv/HMMWV_DoubleWishbone.h"
%include "../../../chrono_models/vehicle/hmmwv/HMMWV_DoubleWishboneReduced.h"

%include "../../../chrono_models/vehicle/sedan/Sedan_DoubleWishbone.h"
%include "../../../chrono_models/vehicle/sedan/Sedan_MultiLink.h"

%include "../../../chrono_models/vehicle/citybus/CityBus_ToeBarLeafspringAxle.h"
%include "../../../chrono_models/vehicle/citybus/CityBus_SolidAxle.h"
%include "../../../chrono_models/vehicle/citybus/CityBus_LeafspringAxle.h"

%include "../../../chrono_models/vehicle/man/MAN_5t_BellcrankSolid3LinkAxle.h"
%include "../../../chrono_models/vehicle/man/MAN_5t_Solid3LinkAxle.h"
%include "../../../chrono_models/vehicle/man/MAN_10t_Front1Axle.h"
%include "../../../chrono_models/vehicle/man/MAN_10t_Front2Axle.h"

%include "../../../chrono_models/vehicle/uaz/UAZBUS_ToeBarLeafspringAxle.h"
%include "../../../chrono_models/vehicle/uaz/UAZBUS_LeafspringAxle.h"

%include "../../../chrono_models/vehicle/gator/Gator_SingleWishbone.h"
%include "../../../chrono_models/vehicle/gator/Gator_RigidSuspension.h"

%include "../../../chrono_models/vehicle/rccar/RCCar_DoubleWishbone.h"

%include "../../../chrono_models/vehicle/feda/FEDA_DoubleWishbone.h"
