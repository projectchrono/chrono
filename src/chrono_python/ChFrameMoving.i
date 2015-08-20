%{

/* Includes the header in the wrapper code */
#include "core/ChFrameMoving.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../core/ChFrameMoving.h"    




%template(ChFrameMovingD) chrono::ChFrameMoving<double>; 
// %template(ChFrameMovingF) chrono::ChFrameMoving<float>; 

