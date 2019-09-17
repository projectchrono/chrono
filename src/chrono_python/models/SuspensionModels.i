%{

/* Includes additional C++ in the wrapper code */

#include <string>
#include <vector>
#include "chrono/core/ChVector.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColorAsset.h"
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


/* Parse the header file to generate wrappers */
%import "ChSuspension.i"

// Model:
%include "../chrono_models/vehicle/generic/Generic_RigidSuspension.h"
%include "../chrono_models/vehicle/generic/Generic_RigidPinnedAxle.h"
%include "../chrono_models/vehicle/generic/Generic_MultiLink.h"
%include "../chrono_models/vehicle/generic/Generic_DoubleWishbone.h"

%include "../chrono_models/vehicle/hmmwv/HMMWV_DoubleWishbone.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_DoubleWishboneReduced.h"


%include "../chrono_models/vehicle/sedan/Sedan_DoubleWishbone.h"
%include "../chrono_models/vehicle/sedan/Sedan_MultiLink.h"

%include "../chrono_models/vehicle/citybus/CityBus_ToeBarLeafspringAxle.h"
%include "../chrono_models/vehicle/citybus/CityBus_SolidAxle.h"
%include "../chrono_models/vehicle/citybus/CityBus_LeafspringAxle.h"