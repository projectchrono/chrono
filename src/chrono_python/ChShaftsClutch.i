%{

/* Includes the header in the wrapper code */
#include "physics/ChShaftsClutch.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChShaftsCouple.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer
%shared_ptr(chrono::ChShaftsClutch)

/* Parse the header file to generate wrappers */
%include "../physics/ChShaftsClutch.h"  







