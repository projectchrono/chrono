%{

/* Includes the header in the wrapper code */
#include "physics/ChShaftsClutch.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChShaftsCouple.i"


/* Parse the header file to generate wrappers */
%include "../physics/ChShaftsClutch.h"  







