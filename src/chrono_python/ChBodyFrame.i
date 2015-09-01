%{

/* Includes the header in the wrapper code */
#include "physics/ChBodyFrame.h"

using namespace chrono;

%}



// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChBodyFrame.h"    


// Define also the shared pointer chrono::ChShared<ChBodyFrame> 
// (renamed as 'ChBodyFrameShared' in python)

%DefChSharedPtr(chrono::,ChBodyFrame)