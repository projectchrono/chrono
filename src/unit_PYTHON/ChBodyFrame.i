%{

/* Includes the header in the wrapper code */
#include "physics/ChBodyFrame.h"

using namespace chrono;

%}

#define ChApi 
%include "../physics/ChVariablesInterface.h" 

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

/* Parse the header file to generate wrappers */
%include "../physics/ChBodyFrame.h"    


// Define also the shared pointer chrono::ChShared<ChBodyFrame> 
// (renamed as 'ChBodyFrameShared' in python)

%DefChSharedPtr(chrono::,ChBodyFrame)