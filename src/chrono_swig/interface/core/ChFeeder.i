%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChFeeder.h"

%}
 
// Forward ref (parent class do not need %import if all .i are included in proper order
//%import "ChFeeder.i"

/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChFeeder.h"  






