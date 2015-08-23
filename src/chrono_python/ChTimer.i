%{

/* Includes the header in the wrapper code */
#include "core/ChTimer.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../chrono/core/ChTimer.h"    



%template(ChTimerD) chrono::ChTimer<double>;  

