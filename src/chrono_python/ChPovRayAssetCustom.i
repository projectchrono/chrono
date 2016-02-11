%{

/* Includes additional C++ in the wrapper code */

%}
 
%import "ChVector.i"
%import "ChVisualization.i"

// Enable shared pointer 
%shared_ptr(chrono::postprocess::ChPovRayAssetCustom)

/* Parse the header file to generate wrappers */
%include "../chrono_postprocess/ChPovRayAssetCustom.h"    


