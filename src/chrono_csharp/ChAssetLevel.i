%{

/* Includes the header in the wrapper code */
#include "chrono/assets/ChAssetLevel.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChAssetLevel)

/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChAssetLevel.h"    



