%{

/* Includes the header in the wrapper code */
#include "chrono/assets/ChVisualization.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChVisualization)

/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChVisualization.h"    



