%{

/* Includes the header in the wrapper code */
#include "chrono_irrlicht/ChIrrNodeAsset.h"

using namespace chrono;

%}
 
%import  "ChAsset.i"

// Enable shared pointer
%shared_ptr(chrono::irrlicht::ChIrrNodeAsset)

/* Parse the header file to generate wrappers */
%include "../chrono_irrlicht/ChIrrNodeAsset.h"    



