%{

/* Includes the header in the wrapper code */
#include "assets/ChColorAsset.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../assets/ChColorAsset.h"    


// Define also the shared pointer chrono::ChShared<ChColorAsset> 
// (renamed as 'ChColorAssetShared' in python)

%DefChSharedPtr(chrono::,ChColorAsset)
