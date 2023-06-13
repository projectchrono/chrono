%{

/* Includes the header in the wrapper code */
#include "chrono/serialization/ChArchive.h"

using namespace chrono;

%}

// Trick to disable a macro that stops SWIG




/* Parse the header file to generate wrappers */
 %include "../../../chrono/serialization/ChArchive.h"    


