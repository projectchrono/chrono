%{

/* Includes the header in the wrapper code */
#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChAssetLevel.h"

using namespace chrono;

%}
 
%shared_ptr(chrono::ChAsset)
%shared_ptr(chrono::ChAssetLevel)

/* Parse the header file to generate wrappers */
%include "../../../chrono/assets/ChAsset.h"    
%include "../../../chrono/assets/ChAssetLevel.h"    


