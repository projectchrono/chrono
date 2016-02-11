%{

/* Includes the header in the wrapper code */
#include "physics/ChBody.h"

%}
 

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Forward ref
//%import "ChPhysicsItem.i"   // (parent class does not need %import if all .i are included in proper order
%import "ChMaterialSurface.i"
%import "ChCollisionModel.i"

// Enable the shared pointer 
%shared_ptr(chrono::ChBody)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChBody.h"  




