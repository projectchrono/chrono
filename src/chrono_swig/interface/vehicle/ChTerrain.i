%{
#include <string>
#include <vector>

#include "chrono/core/ChVector2.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChFrame.h"

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColormap.h"
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

%include "chrono_swig/interface/core/ChGeometry.i"
%include "chrono_swig/interface/core/ChColor.i"
%include "chrono_swig/interface/core/ChColormap.i"
%include "chrono_swig/interface/core/ChSystem.i"
%include "chrono_swig/interface/core/ChVector2.i"
%include "chrono_swig/interface/core/ChVector3.i"
%include "chrono_swig/interface/core/ChFrame.i"
%include "chrono_swig/interface/core/ChBody.i"
%include "chrono_swig/interface/core/ChNodeXYZ.i"
%include "chrono_swig/interface/core/ChLoadContainer.i"
%include "chrono_swig/interface/core/ChVisualShape.i"

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
#ifdef CHRONO_FSI_SPH
%include "../../../chrono_vehicle/terrain/CRMTerrain.h"
#endif
#endif             // --------------------------------------------------------------------- PYTHON

#if defined(SWIGCSHARP) && defined(HAVE_OPENCRG)
%include "../../../chrono_vehicle/terrain/CRGTerrain.h"
#endif
