%{

/* Includes the header in the wrapper code */
#include "physics/ChLimit.h"

%}
 
// Tell SWIG about parent class in Python
//%import "..."



// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../physics/ChLimit.h"  



// Define also the shared pointer chrono::ChShared<ChXxxx> 
// (renamed as 'ChXxxxShared' in python)

//%DefChSharedPtr(chrono::,ChLinkLimit)



