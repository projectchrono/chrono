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
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_vehicle/chassis/ChChassisConnectorArticulated.h"
#include "chrono_vehicle/chassis/ChChassisConnectorTorsion.h"
#include "chrono_vehicle/chassis/ChChassisConnectorHitch.h"
#include "chrono_vehicle/chassis/RigidChassis.h"
#include "chrono_vehicle/chassis/ChassisConnectorArticulated.h"
#include "chrono_vehicle/chassis/ChassisConnectorHitch.h"
#include "chrono_vehicle/chassis/ChassisConnectorTorsion.h"

#include "chrono_thirdparty/rapidjson/document.h"

%}

%shared_ptr(chrono::vehicle::ChChassis)
%shared_ptr(chrono::vehicle::ChChassisRear)
%shared_ptr(chrono::vehicle::ChRigidChassis)
%shared_ptr(chrono::vehicle::ChRigidChassisRear)
%shared_ptr(chrono::vehicle::ChChassisConnector)
%shared_ptr(chrono::vehicle::ChChassisConnectorArticulated)
%shared_ptr(chrono::vehicle::ChChassisConnectorTorsion)
%shared_ptr(chrono::vehicle::ChChassisConnectorHitch)
%shared_ptr(chrono::vehicle::RigidChassis)
%shared_ptr(chrono::vehicle::RigidChassisRear)
%shared_ptr(chrono::vehicle::ChassisConnectorArticulated)
%shared_ptr(chrono::vehicle::ChassisConnectorHitch)
%shared_ptr(chrono::vehicle::ChassisConnectorTorsion)


%import "../../chrono/assets/ChAsset.h"
%import "../../chrono/assets/ChAssetLevel.h"


/* Parse the header file to generate wrappers */
%include "../../chrono_vehicle/ChChassis.h"
%include "../../chrono_vehicle/chassis/ChRigidChassis.h"
%include "../../chrono_vehicle/chassis/ChChassisConnectorArticulated.h"
%include "../../chrono_vehicle/chassis/ChChassisConnectorTorsion.h"
%include "../../chrono_vehicle/chassis/ChChassisConnectorHitch.h"
%include "../../chrono_vehicle/chassis/RigidChassis.h"
%include "../../chrono_vehicle/chassis/ChassisConnectorArticulated.h"
%include "../../chrono_vehicle/chassis/ChassisConnectorHitch.h"
%include "../../chrono_vehicle/chassis/ChassisConnectorTorsion.h"

%include "chrono_csharp/models/ChassisModels.i"
