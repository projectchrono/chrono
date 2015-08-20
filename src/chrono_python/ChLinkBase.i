%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkBase.h"

%}

// Forward ref 
//%import "ChPhysicsItem.i" // parent class does not need %import if all .i are included in proper order

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

/* Parse the header file to generate wrappers */
%include "../physics/ChLinkBase.h"  



// Define also the shared pointer chrono::ChShared<ChLinkBase> 
// (renamed as 'ChLinkBaseShared' in python)

%DefChSharedPtr(chrono::,ChLinkBase)


