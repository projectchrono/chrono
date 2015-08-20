%{

/* Includes the header in the wrapper code */
#include "assets/ChAsset.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../assets/ChAsset.h"    

// Define also the shared pointer chrono::ChShared<ChAsset> 
// (renamed as 'ChAssetShared' in python)

%DefChSharedPtr(chrono::,ChAsset)
