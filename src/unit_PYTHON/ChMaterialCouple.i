%{

/* Includes the header in the wrapper code */
#include "physics/ChMaterialCouple.h"

using namespace chrono;

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

/* Parse the header file to generate wrappers */
%include "../physics/ChMaterialCouple.h"    


