%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChBody.h"

%}
 

// Forward ref
//%import "ChPhysicsItem.i"   // (parent class does not need %import if all .i are included in proper order
%import "ChMaterialSurfaceNSC.i"
%import "ChCollisionModel.i"

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChBody.h"  




