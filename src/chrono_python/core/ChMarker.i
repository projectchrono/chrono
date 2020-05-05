%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChMarker.h"


%}
 
// Forward ref 
//%import "ChPhysicsItem.i" // parent class does not need %import if all .i are included in proper order
// For some strange reason, the forward reference in the .h must be replicated here:
namespace chrono { class ChBody; }


/* Parse the header file to generate wrappers */
%include "../../chrono/physics/ChMarker.h"  




