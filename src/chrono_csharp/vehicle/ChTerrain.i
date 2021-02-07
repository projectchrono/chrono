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
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include "chrono_thirdparty/rapidjson/document.h"

%}



%feature("director") chrono::vehicle::ChTerrain;
%feature("director") SoilParametersCallback;


%import "chrono_csharp/core/ChColor.i"
%import "chrono_csharp/core/ChSystem.i"
%import "chrono_csharp/core/ChVector.i"
%import "chrono_csharp/core/ChBody.i"
%import "chrono_csharp/core/ChLoadContainer.i"
%import "../../chrono/assets/ChTriangleMeshShape.h"
%import "chrono_csharp/core/ChColorAsset.i"

%shared_ptr(chrono::vehicle::ChTerrain)
%shared_ptr(chrono::vehicle::FlatTerrain)
%shared_ptr(chrono::vehicle::RigidTerrain::Patch)
%shared_ptr(chrono::vehicle::RigidTerrain)
%shared_ptr(chrono::vehicle::SCMDeformableSoil)
%shared_ptr(chrono::vehicle::SCMDeformableTerrain)
%shared_ptr(chrono::vehicle::SCMDeformableTerrain::SoilParametersCallback)

/* Parse the header file to generate wrappers */
%include "../../chrono_vehicle/ChTerrain.h"    
%include "../../chrono_vehicle/terrain/FlatTerrain.h"
%include "../../chrono_vehicle/terrain/RigidTerrain.h"
%include "../../chrono_vehicle/terrain/SCMDeformableTerrain.h"


//%include "../../chrono_vehicle/terrain/CRGTerrain.h"
