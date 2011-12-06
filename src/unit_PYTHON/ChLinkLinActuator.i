%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkLinActuator.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLinkLock.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../physics/ChLinkLinActuator.h"  



// Define also the shared pointer chrono::ChShared<ChXxxx> 
// (renamed as 'ChXxxxShared' in python)

%DefChSharedPtr(ChLinkLinActuatorShared, ChLinkLinActuator)



