%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChFrameMoving.h"

using namespace chrono;

%}


/* Parse the header file to generate wrappers */
 %include "../../../chrono/core/ChFrameMoving.h"    




%template(ChFrameMovingd) chrono::ChFrameMoving<double>; 
// %template(ChFrameMovingf) chrono::ChFrameMoving<float>; 

