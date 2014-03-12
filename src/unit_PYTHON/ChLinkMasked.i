%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkMasked.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLinkMarkers.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file(s) to generate wrappers */
%include "../physics/ChLinkforce.h"  
%include "../physics/ChLinkMasked.h"  



// Define also the shared pointer chrono::ChShared<ChXxxx> 
// (renamed as 'ChXxxxShared' in python)

%DefChSharedPtr(chrono::,ChLinkMasked)



