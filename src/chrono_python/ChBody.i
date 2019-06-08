%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChBody.h"

%}
 
%shared_ptr(chrono::ChBody)

// Forward ref
//%import "ChPhysicsItem.i"   // (parent class does not need %import if all .i are included in proper order
%import "ChMaterialSurface.i"
%import "ChCollisionModel.i"

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChBody.h"  




