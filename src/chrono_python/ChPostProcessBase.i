%{

/* Includes additional C++ in the wrapper code */

%}
 
// Forward ref
%import "ChSystem.i"

/* Parse the header file to generate wrappers */
%include "../chrono_postprocess/ChPostProcessBase.h"    

// Define also the shared pointer chrono::ChShared<thisclass> 

//%DefChSharedPtr(chrono::,ChPostProcessBase)
