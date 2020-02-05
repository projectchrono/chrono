%{

/* Includes the header in the wrapper code */
#include "chrono/assets/ChAsset.h"

using namespace chrono;

%}
 
%shared_ptr(chrono::ChAsset)

/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChAsset.h"    


