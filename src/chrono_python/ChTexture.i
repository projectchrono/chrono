%{

/* Includes the header in the wrapper code */
#include "chrono/assets/ChTexture.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChTexture)

/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChTexture.h"    



