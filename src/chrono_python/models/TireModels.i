%{

/* Includes additional C++ in the wrapper code */

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_models/ChApiModels.h"

#include "chrono_models/vehicle/generic/Generic_RigidTire.h"
#include "chrono_models/vehicle/generic/Generic_RigidMeshTire.h"
#include "chrono_models/vehicle/generic/Generic_FialaTire.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_ReissnerTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac02Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_LugreTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_FialaTire.h"

#include "chrono_models/vehicle/sedan/Sedan_TMeasyTire.h"
#include "chrono_models/vehicle/sedan/Sedan_RigidTire.h"
%}



%import "ChTire.i"

%shared_ptr(chrono::vehicle::generic::Generic_RigidTire)
%shared_ptr(chrono::vehicle::generic::Generic_RigidMeshTire)
%shared_ptr(chrono::vehicle::generic::Generic_FialaTire)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_RigidTire)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_ReissnerTire)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Pac89Tire)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Pac02Tire)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_LugreTire)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_FialaTire)
%shared_ptr(chrono::vehicle::sedan::Sedan_TMeasyTire)
%shared_ptr(chrono::vehicle::sedan::Sedan_RigidTire)

// Model:

%include "../chrono_models/vehicle/generic/Generic_RigidTire.h"
%include "../chrono_models/vehicle/generic/Generic_RigidMeshTire.h"
%include "../chrono_models/vehicle/generic/Generic_FialaTire.h"

%include "../chrono_models/vehicle/hmmwv/HMMWV_RigidTire.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_ReissnerTire.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_Pac89Tire.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_Pac02Tire.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_LugreTire.h"
%include "../chrono_models/vehicle/hmmwv/HMMWV_FialaTire.h"

%include "../chrono_models/vehicle/sedan/Sedan_TMeasyTire.h"
%include "../chrono_models/vehicle/sedan/Sedan_RigidTire.h"