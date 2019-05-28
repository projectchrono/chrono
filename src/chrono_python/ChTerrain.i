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

#include "chrono_thirdparty/rapidjson/document.h"

%}

// Used in Rigid Terrain, already imported in ChModuleVehicle
//%import(module = "pychrono.core")  "ChSystem.i"
//%import(module = "pychrono.core")  "ChCoordsys.i"

%import "ChVector.i"
%import "ChVisualization.i"

%shared_ptr(chrono::vehicle::RigidTerrain::Patch)

/* Parse the header file to generate wrappers */
%include "../chrono_vehicle/ChTerrain.h"    
%include "../chrono_vehicle/terrain/FlatTerrain.h"
%include "../chrono_vehicle/terrain/RigidTerrain.h"
