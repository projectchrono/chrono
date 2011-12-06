%{

/* Includes the header in the wrapper code */
#include "physics/ChShaftsTorsionSpring.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChShaftsCouple.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../physics/ChShaftsTorsionSpring.h"  



// Define also the shared pointer chrono::ChShared<ChXxxx> 
// (renamed as 'ChXxxxShared' in python)

%DefChSharedPtr(ChShaftsTorsionSpringShared, ChShaftsTorsionSpring)



