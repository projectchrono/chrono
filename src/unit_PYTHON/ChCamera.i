%{

/* Includes the header in the wrapper code */
#include "assets/ChCamera.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
%include "../assets/ChCamera.h"    

// Define also the shared pointer 

%DefChSharedPtr(chrono::,ChCamera)

