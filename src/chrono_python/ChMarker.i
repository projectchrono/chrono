%{

/* Includes the header in the wrapper code */
#include "physics/ChMarker.h"


%}
 
// Forward ref 
//%import "ChPhysicsItem.i" // parent class does not need %import if all .i are included in proper order
// For some strange reason, the forward reference in the .h must be replicated here:
namespace chrono { class ChBody; }

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../physics/ChMarker.h"  


// Define also the shared pointer chrono::ChShared<ChForce> 
// (renamed as 'ChForceShared' in python)

%DefChSharedPtr(chrono::,ChMarker)

