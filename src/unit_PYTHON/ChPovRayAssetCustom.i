%{

/* Includes additional C++ in the wrapper code */

%}
 
%import "ChVector.i"
%import "ChVisualization.i"


/* Parse the header file to generate wrappers */
%include "../unit_POSTPROCESS/ChPovRayAssetCustom.h"    


// Define also the shared pointer 
//%DefChSharedPtr3(mynamespace::, myclassname)

%DefChSharedPtr3(chrono::postprocess::,ChPovRayAssetCustom)