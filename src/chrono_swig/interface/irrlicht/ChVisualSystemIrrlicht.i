
////#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP
////%csmethodmodifiers chrono::ChVisualSystem::BindItem "public"
////%csmethodmodifiers chrono::ChVisualSystem::AddVisualModel "public"
////%csmethodmodifiers chrono::ChVisualSystem::UpdateVisualModel "public"
////%csmethodmodifiers chrono::ChVisualSystem::RenderFrame "public"
////%csmethodmodifiers chrono::ChVisualSystemIrrlicht::AddGrid(double, double, int, int, ChCoordsysd, ChColor) "public override"
////
////#endif // --------------------------------------------------------------------- CSHARP

%{
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_irrlicht/ChIrrNodeShape.h"
#include "chrono_irrlicht/ChIrrNodeModel.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;
%}

%shared_ptr(chrono::irrlicht::ChVisualSystemIrrlicht)

%import(module="pychrono.core") "chrono_swig/interface/core/ChVisualSystem.i"

%include "../../../chrono_irrlicht/ChIrrNodeShape.h"    
%include "../../../chrono_irrlicht/ChIrrNodeModel.h"    
%include "../../../chrono_irrlicht/ChVisualSystemIrrlicht.h"

%DefSharedPtrDynamicCast2NS(chrono, chrono::irrlicht, ChVisualSystem, ChVisualSystemIrrlicht)
