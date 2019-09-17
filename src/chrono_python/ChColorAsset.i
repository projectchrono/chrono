%{

/* Includes the header in the wrapper code */
#include "chrono/assets/ChColorAsset.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChColorAsset)

/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChColorAsset.h"    



