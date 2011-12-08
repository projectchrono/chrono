%{

/* Includes the header in the wrapper code */
#include "physics/ChConveyor.h"

%}
 
// Forward ref (parent class do not need %import if all .i are included in proper order
//%import "ChBody.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../physics/ChConveyor.h"  



// Define also the shared pointer chrono::ChShared<ChConveyor> 
// (renamed as 'ChConveyorShared' in python)

%DefChSharedPtr(ChConveyorShared, ChConveyor)


