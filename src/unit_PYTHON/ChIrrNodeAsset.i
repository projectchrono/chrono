%{

/* Includes the header in the wrapper code */
#include "irrlicht_interface/ChIrrNodeAsset.h"

using namespace chrono;

%}
 
%import  "ChAsset.i"

/* Parse the header file to generate wrappers */
 %include "../irrlicht_interface/ChIrrNodeAsset.h"    

// Define also the shared pointer chrono::ChShared<ChIrrNodeAsset> 
// (renamed as 'ChIrrNodeAssetShared' in python)

%DefChSharedPtr(chrono::,ChIrrNodeAsset)

