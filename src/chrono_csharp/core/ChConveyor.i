%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChConveyor.h"

%}
 
// Forward ref (parent class do not need %import if all .i are included in proper order
//%import "ChBody.i"

/* Parse the header file to generate wrappers */
%include "../../chrono/physics/ChConveyor.h"  






