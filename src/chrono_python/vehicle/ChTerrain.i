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

%import(module = "pychrono.core") "chrono_python/core/ChColor.i"
%import(module = "pychrono.core") "chrono_python/core/ChSystem.i"
%import(module = "pychrono.core") "chrono_python/core/ChVector.i"
%import(module = "pychrono.core") "chrono_python/core/ChBody.i"
%import(module = "pychrono.core") "chrono_python/core/ChLoadContainer.i"
%import(module = "pychrono.core") "../../chrono/assets/ChTriangleMeshShape.h"
%import(module = "pychrono.core") "chrono_python/core/ChColorAsset.i"

%shared_ptr(chrono::vehicle::RigidTerrain::Patch)
%shared_ptr(chrono::vehicle::SCMDeformableSoil)
%shared_ptr(chrono::vehicle::SCMDeformableTerrain)
%shared_ptr(chrono::vehicle::SCMDeformableTerrain::SoilParametersCallback)

/* Parse the header file to generate wrappers */
%include "../../chrono_vehicle/ChTerrain.h"    
%include "../../chrono_vehicle/terrain/FlatTerrain.h"
%include "../../chrono_vehicle/terrain/RigidTerrain.h"

%feature("director") SoilParametersCallback;
%include "../../chrono_vehicle/terrain/SCMDeformableTerrain.h"

//%include "../../chrono_vehicle/terrain/CRGTerrain.h"
//FEADeformableTerrain