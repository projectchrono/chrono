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
/*#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishbone.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChMacPhersonStrut.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChLeafspringAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChHendricksonPRIMAXX.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishboneReduced.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChMultiLink""
#include "chrono_vehicle/wheeled_vehicle/suspension/ChRigidPinnedAxle""
#include "chrono_vehicle/wheeled_vehicle/suspension/ChSemiTrailingArm""
#include "chrono_vehicle/wheeled_vehicle/suspension/ChRigidSuspension""
#include "chrono_vehicle/wheeled_vehicle/suspension/ChSolidAxle""
#include "chrono_vehicle/wheeled_vehicle/suspension/ChThreeLinkIRS""
#include "chrono_vehicle/wheeled_vehicle/suspension/ChToeBarLeafspringAxle""
#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishbone""
#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishboneReduced""
#include "chrono_vehicle/wheeled_vehicle/suspension/HendricksonPRIMAXX""
#include "chrono_vehicle/wheeled_vehicle/suspension/LeafspringAxle"
#include "chrono_vehicle/wheeled_vehicle/suspension/MacPhersonStrut"
#include "chrono_vehicle/wheeled_vehicle/suspension/MultiLink"
#include "chrono_vehicle/wheeled_vehicle/suspension/SemiTrailingArm"
#include "chrono_vehicle/wheeled_vehicle/suspension/SolidAxle"
#include "chrono_vehicle/wheeled_vehicle/suspension/ThreeLinkIRS"
#include "chrono_vehicle/wheeled_vehicle/suspension/ToeBarLeafspringAxle"*/

#include "chrono_models/vehicle/generic/Generic_RigidSuspension.h"
#include "chrono_models/vehicle/generic/Generic_RigidPinnedAxle.h"
#include "chrono_models/vehicle/generic/Generic_MultiLink.h"
#include "chrono_models/vehicle/generic/Generic_DoubleWishbone.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_DoubleWishbone.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_DoubleWishboneReduced.h"

#include "chrono_models/vehicle/sedan/Sedan_DoubleWishbone.h"
#include "chrono_models/vehicle/sedan/Sedan_MultiLink.h"

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