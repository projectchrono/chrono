%{
#include "chrono/assets/ChVisualMaterial.h"
using namespace chrono;
%}


%shared_ptr(chrono::ChVisualMaterial)

/* Parse the header file to generate wrappers */
%include "../../../chrono/assets/ChVisualMaterial.h"
