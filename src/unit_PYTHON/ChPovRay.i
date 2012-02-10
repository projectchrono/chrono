%{

/* Includes additional C++ in the wrapper code */

%}
 

/* Parse the header file to generate wrappers */
%include "../unit_POSTPROCESS/ChPovRay.h"    


// Define also the shared pointer chrono::ChShared<thisclass> 

//%DefChSharedPtr(ChPovRayShared, ChPovRay)
