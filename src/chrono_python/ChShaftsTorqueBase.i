%{

/* Includes the header in the wrapper code */
#include "physics/ChShaftsTorqueBase.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChShaftsCouple.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer 
%shared_ptr(chrono::ChShaftsTorqueBase)

/* Parse the header file to generate wrappers */
%include "../physics/ChShaftsTorqueBase.h"  







