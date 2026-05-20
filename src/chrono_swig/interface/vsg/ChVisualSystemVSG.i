%{
#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vsg3d;
%}

#ifdef SWIGCSHARP
%csmethodmodifiers chrono::vsg3d::ChVisualSystemVSG::AddGrid "public new"
#endif

%shared_ptr(chrono::vsg3d::ChVisualSystemVSG)
%shared_ptr(chrono::vsg3d::ChVisualSystemVSGPlugin)

%import(module="pychrono.core") "chrono_swig/interface/core/ChVisualSystem.i"

%include "../../../chrono_vsg/ChVisualSystemVSG.h"    

%DefSharedPtrDynamicCast2NS(chrono, chrono::vsg3d, ChVisualSystem, ChVisualSystemVSG)
