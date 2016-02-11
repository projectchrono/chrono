%{

/* Includes the header in the wrapper code */
#include "assets/ChVisualization.h"

using namespace chrono;

%}

// Define also the shared pointer 
%shared_ptr(chrono::ChVisualization)

/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChVisualization.h"    



