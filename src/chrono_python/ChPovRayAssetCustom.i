%{

/* Includes additional C++ in the wrapper code */

%}
 
%import "ChVector.i"
%import "ChVisualization.i"


/* Parse the header file to generate wrappers */
%include "../unit_POSTPROCESS/ChPovRayAssetCustom.h"    


// Define also the shared pointer 
//%DefChSharedPtr(mynamespace::, myclassname)

%DefChSharedPtr(chrono::postprocess::,ChPovRayAssetCustom)