%{
#include <string>
#include <vector>

#include "chrono/core/ChVector2.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChFrame.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColormap.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"
#if defined(SWIGCSHARP) && defined(HAVE_OPENCRG)
    #include "chrono_vehicle/terrain/CRGTerrain.h"
#endif

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON
#ifdef CHRONO_FSI_SPH
#include "chrono_vehicle/terrain/CRMTerrain.h"
#endif
#endif             // --------------------------------------------------------------------- PYTHON

#include "chrono_thirdparty/rapidjson/document.h"
%}

#ifdef SWIGCSHARP
%import "chrono_swig/interface/core/ChColor.i"
%import "chrono_swig/interface/core/ChColormap.i"
%import "chrono_swig/interface/core/ChSystem.i"
%import "chrono_swig/interface/core/ChVector2.i"
%import "chrono_swig/interface/core/ChVector3.i"
%import "chrono_swig/interface/core/ChFrame.i"
%import "chrono_swig/interface/core/ChBody.i"
%import "chrono_swig/interface/core/ChNodeXYZ.i"
%import "chrono_swig/interface/core/ChLoadContainer.i"
%import "../../../chrono/assets/ChVisualShapeTriangleMesh.h"
#endif

#ifdef SWIGPYTHON
%import(module = "pychrono.core") "chrono_swig/interface/core/ChColor.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChColormap.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChSystem.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChVector2.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChVector3.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChFrame.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChBody.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChNodeXYZ.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChLoadContainer.i"
%import(module = "pychrono.core") "../../../chrono/assets/ChVisualShapeTriangleMesh.h"
#endif

%shared_ptr(chrono::vehicle::ChTerrain)
%shared_ptr(chrono::vehicle::FlatTerrain)
%shared_ptr(chrono::vehicle::RigidTerrain::Patch)
%shared_ptr(chrono::vehicle::RigidTerrain)
%shared_ptr(chrono::vehicle::SCMLoader)
%shared_ptr(chrono::vehicle::SCMTerrain)
%shared_ptr(chrono::vehicle::SCMTerrain::SoilParametersCallback)

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON
#ifdef CHRONO_FSI_SPH
%shared_ptr(chrono::vehicle::CRMTerrain)
#endif
#endif             // --------------------------------------------------------------------- PYTHON

#if defined(SWIGCSHARP) && defined(HAVE_OPENCRG)
%shared_ptr(chrono::vehicle::CRGTerrain)
#endif

%template(ChPatchList) std::vector<std::shared_ptr<chrono::vehicle::RigidTerrain::Patch>>;
%template(ChSCMTerrainNodeLevel) std::pair<chrono::ChVector2i, double>;
%template(ChSCMTerrainNodeLevelList) std::vector<std::pair<chrono::ChVector2i, double>>; // To support SCMTerrain::Get/SetModifiedNodes

// Parse the header file to generate wrappers
%include "../../../chrono_vehicle/ChTerrain.h"    
%include "../../../chrono_vehicle/terrain/FlatTerrain.h"
%include "../../../chrono_vehicle/terrain/RigidTerrain.h"

%feature("director") chrono::vehicle::ChTerrain;
%feature("director") SoilParametersCallback;
%include "cpointer.i"
%pointer_functions(int, intp)
%pointer_functions(double, doublep)
%include "../../../chrono_vehicle/terrain/SCMTerrain.h"

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON
#ifdef CHRONO_FSI
%include "../../../chrono_vehicle/terrain/CRMTerrain.h"
#endif
#endif             // --------------------------------------------------------------------- PYTHON

#if defined(SWIGCSHARP) && defined(HAVE_OPENCRG)
%include "../../../chrono_vehicle/terrain/CRGTerrain.h"
#endif
