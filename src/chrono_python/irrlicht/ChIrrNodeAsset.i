%{

/* Includes the header in the wrapper code */
#include "chrono_irrlicht/ChIrrNodeAsset.h"

using namespace chrono;

%}
 
%import  "chrono_python/core/ChAsset.i"

/* Parse the header file to generate wrappers */
%include "../../chrono_irrlicht/ChIrrNodeAsset.h"    



