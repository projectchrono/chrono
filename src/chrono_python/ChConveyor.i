%{

/* Includes the header in the wrapper code */
#include "physics/ChConveyor.h"

%}
 
// Forward ref (parent class do not need %import if all .i are included in proper order
//%import "ChBody.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer
%shared_ptr(chrono::ChConveyor)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChConveyor.h"  






