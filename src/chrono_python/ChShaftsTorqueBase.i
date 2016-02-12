%{

/* Includes the header in the wrapper code */
#include "physics/ChShaftsTorqueBase.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChShaftsCouple.i"


/* Parse the header file to generate wrappers */
%include "../physics/ChShaftsTorqueBase.h"  







