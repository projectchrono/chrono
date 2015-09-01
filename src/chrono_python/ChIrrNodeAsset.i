%{

/* Includes the header in the wrapper code */
#include "chrono_irrlicht/ChIrrNodeAsset.h"

using namespace chrono;

%}
 
%import  "ChAsset.i"

/* Parse the header file to generate wrappers */
 %include "../chrono_irrlicht/ChIrrNodeAsset.h"    

// Define also the shared pointer chrono::ChShared<ChIrrNodeAsset> 
// (renamed as 'ChIrrNodeAssetShared' in python)

%DefChSharedPtr(chrono::,ChIrrNodeAsset)

