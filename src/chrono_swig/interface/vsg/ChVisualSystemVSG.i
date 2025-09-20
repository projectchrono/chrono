%{

/* Includes the header in the wrapper code */
#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vsg3d;

%}

%shared_ptr(chrono::vsg3d::ChVisualSystemVSG)
%shared_ptr(chrono::vsg3d::ChVisualSystemVSGPlugin)

/* Parse the header file to generate wrappers */
%include "../../../chrono_vsg/ChVisualSystemVSG.h"    

%DefSharedPtrDynamicCast2NS(chrono, chrono::vsg3d, ChVisualSystem, ChVisualSystemVSG)