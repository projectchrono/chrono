%{

/* Includes the header in the wrapper code */
#include "chrono/utils/ChOpenMP.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChOMP) 

/* Parse the header file to generate wrappers */
%include "../../../chrono/utils/ChOpenMP.h" 






