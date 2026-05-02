%{
#include "chrono_irrlicht/ChIrrNodeShape.h"
#include "chrono_irrlicht/ChIrrNodeModel.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;
%}

#ifdef SWIGCSHARP
%csmethodmodifiers chrono::irrlicht::ChVisualSystemIrrlicht::AddGrid "public new"
#endif

%shared_ptr(chrono::irrlicht::ChVisualSystemIrrlicht)

%import(module="pychrono.core") "chrono_swig/interface/core/ChVisualSystem.i"

%include "../../../chrono_irrlicht/ChIrrNodeShape.h"    
%include "../../../chrono_irrlicht/ChIrrNodeModel.h"    
%include "../../../chrono_irrlicht/ChVisualSystemIrrlicht.h"

%DefSharedPtrDynamicCast2NS(chrono, chrono::irrlicht, ChVisualSystem, ChVisualSystemIrrlicht)
