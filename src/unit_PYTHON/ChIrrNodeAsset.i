%{

/* Includes the header in the wrapper code */
#include "unit_IRRLICHT/ChIrrNodeAsset.h"

using namespace chrono;

%}
 
%import  "ChAsset.i"

/* Parse the header file to generate wrappers */
 %include "../unit_IRRLICHT/ChIrrNodeAsset.h"    

// Define also the shared pointer chrono::ChShared<ChIrrNodeAsset> 
// (renamed as 'ChIrrNodeAssetShared' in python)

%DefChSharedPtr(chrono::,ChIrrNodeAsset)

