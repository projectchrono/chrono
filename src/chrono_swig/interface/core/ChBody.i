%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChBody.h"

%}

%template(ChForceList) std::vector< std::shared_ptr<chrono::ChForce> >;
%template(ChMarkerList) std::vector< std::shared_ptr<chrono::ChMarker> >;
 
%shared_ptr(chrono::ChBody)

// Forward ref
//%import "ChPhysicsItem.i"   // (parent class does not need %import if all .i are included in proper order
%import "chrono_swig/interface/core/ChMaterialSurface.i"
%import "chrono_swig/interface/core/ChCollisionModel.i"
%import "chrono_swig/interface/core/ChMarker.i"

/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChBody.h"  




