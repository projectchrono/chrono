%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkMarkers.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLink.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../physics/ChLinkMarkers.h"  



// Define also the shared pointer chrono::ChShared<ChXxxx> 
// (renamed as 'ChXxxxShared' in python)

%DefChSharedPtr(chrono::,ChLinkMarkers)



