%{

/* Includes the header in the wrapper code */
#include "physics/ChShaftsThermalEngine.h"

%}
 
// Tell SWIG about parent class in Python
//%import "ChShaftsTorqueBase.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer 
%shared_ptr(chrono::ChShaftsThermalEngine)

/* Parse the header file to generate wrappers */
%include "../physics/ChShaftsThermalEngine.h"  







