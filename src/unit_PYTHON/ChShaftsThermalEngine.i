%{

/* Includes the header in the wrapper code */
#include "physics/ChShaftsThermalEngine.h"

%}
 
// Tell SWIG about parent class in Python
//%import "ChShaftsTorqueBase.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../physics/ChShaftsThermalEngine.h"  



// Define also the shared pointer chrono::ChShared<ChXxxx> 
// (renamed as 'ChXxxxShared' in python)

%DefChSharedPtr(chrono::,ChShaftsThermalEngine)



