%{

/* Includes the header in the wrapper code */
#include "physics/ChForce.h"

%}
 
%import "ChForce.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../physics/ChForce.h"  



// Define also the object for the shared pointer
%include "../core/ChSmartpointers.h"
%template(ChSharedForce) chrono::ChSharedPtr<ChForce>;

