%{

/* Includes additional C++ in the wrapper code */

%}
 
%import "ChVector.i"
%import "ChVisualization.i"


/* Parse the header file to generate wrappers */
%include "../chrono_postprocess/ChPovRay.h"    


// Define also the shared pointer chrono::ChShared<thisclass> 

//%DefChSharedPtr(chrono::,ChPovRay)
