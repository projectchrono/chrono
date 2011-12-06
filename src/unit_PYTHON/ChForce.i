%{

/* Includes the header in the wrapper code */
#include "physics/ChForce.h"

%}
 
%import "ChForce.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../physics/ChForce.h"  



// Define also the shared pointer chrono::ChShared<ChForce> 
// (renamed as 'ChForceShared' in python)

%DefChSharedPtr(ChForceShared, ChForce)


