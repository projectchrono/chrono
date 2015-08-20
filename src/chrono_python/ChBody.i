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


/* Parse the header file to generate wrappers */
%include "../physics/ChBody.h"  


// Define also the shared pointer chrono::ChShared<ChBody> 
// (renamed as 'ChBodyShared' in python)

%DefChSharedPtr(chrono::,ChBody)

