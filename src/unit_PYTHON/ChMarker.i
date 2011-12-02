%{

/* Includes the header in the wrapper code */
#include "physics/ChMarker.h"

%}
 
%import "ChPhysicsItem.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../physics/ChMarker.h"  



// Define also the object for the shared pointer
%include "../core/ChSmartpointers.h"
%template(ChSharedMarker) chrono::ChSharedPtr<ChMarker>;

