%{

/* Includes the header in the wrapper code */
#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vsg3d;

%}

#ifdef SWIGCSHARP
// Mark override method for handling in C#
%csmethodmodifiers chrono::vsg3d::ChVisualSystemVSG::AddGrid "public override"
#endif

%shared_ptr(chrono::vsg3d::ChVisualSystemVSG)
%shared_ptr(chrono::vsg3d::ChVisualSystemVSGPlugin)

// Import base class ChVisualSystem (same pattern as in ChVisualSystemIrrlicht.i)
%import(module="pychrono.core") "chrono_swig/interface/core/ChVisualSystem.i"

/* Parse the header file to generate wrappers */
%include "../../../chrono_vsg/ChVisualSystemVSG.h"    

%DefSharedPtrDynamicCast2NS(chrono, chrono::vsg3d, ChVisualSystem, ChVisualSystemVSG)