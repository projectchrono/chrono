%{

/* Includes additional C++ in the wrapper code */

%}
 
// Forward ref
%import "ChSystem.i"

// Enable shared pointer 
%shared_ptr(chrono::postprocess::ChPostProcessBase)

/* Parse the header file to generate wrappers */
%include "../chrono_postprocess/ChPostProcessBase.h"    


