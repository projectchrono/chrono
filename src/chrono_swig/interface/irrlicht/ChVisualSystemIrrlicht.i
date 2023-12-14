
#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP
%csmethodmodifiers chrono::ChVisualSystem::BindItem "public"
%csmethodmodifiers chrono::ChVisualSystem::AddVisualModel "public"
%csmethodmodifiers chrono::ChVisualSystem::UpdateVisualModel "public"
%csmethodmodifiers chrono::ChVisualSystem::RenderFrame "public"
%csmethodmodifiers chrono::ChVisualSystemIrrlicht::AddGrid(double, double, int, int, ChCoordsysD, ChColor) "public override"

#endif // --------------------------------------------------------------------- CSHARP


%{
#include "chrono_irrlicht/ChIrrNodeShape.h"
#include "chrono_irrlicht/ChIrrNodeModel.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;
%}

%shared_ptr(chrono::irrlicht::ChVisualSystemIrrlicht)

#ifdef SWIGPYTHON
%import(module = "pychrono.core") "chrono_swig/interface/core/ChVisualSystem.i"

#endif

#ifdef SWIGCSHARP
%shared_ptr(chrono::ChVisualSystem)
%import "../../../chrono/assets/ChVisualSystem.h"
// Could possibly avoid the two lines above with just an import of the interface file??
//%import "../../../chrono_swig/interface/core/ChVisualSystem.i"
#endif

%include "../../../chrono_irrlicht/ChIrrNodeShape.h"    
%include "../../../chrono_irrlicht/ChIrrNodeModel.h"    
%include "../../../chrono_irrlicht/ChVisualSystemIrrlicht.h"

%DefSharedPtrDynamicDowncast2NS(chrono, chrono::irrlicht, ChVisualSystem, ChVisualSystemIrrlicht)
