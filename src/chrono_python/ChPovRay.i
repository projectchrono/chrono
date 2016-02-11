%{

/* Includes additional C++ in the wrapper code */

%}
 
%import "ChVector.i"
%import "ChVisualization.i"

// Define also the shared pointer
%shared_ptr(chrono::postprocess::ChPovRay)

/* Parse the header file to generate wrappers */
%include "../chrono_postprocess/ChPovRay.h"    



