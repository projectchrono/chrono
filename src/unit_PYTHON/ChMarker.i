%{

/* Includes the header in the wrapper code */
#include "physics/ChMarker.h"

%}
 
// Forward ref (parent class does not need %import if all .i are included in proper order
//%import "ChPhysicsItem.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../physics/ChMarker.h"  


// Define also the shared pointer chrono::ChShared<ChForce> 
// (renamed as 'ChForceShared' in python)

%DefChSharedPtr(ChMarkerShared, ChMarker)

