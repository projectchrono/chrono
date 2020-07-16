%{

/* Includes the header in the wrapper code */

#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
using namespace chrono;

%};

%shared_ptr(chrono::ChMaterialSurface)
%shared_ptr(chrono::ChMaterialSurfaceNSC)
%shared_ptr(chrono::ChMaterialSurfaceSMC)
%shared_ptr(chrono::ChMaterialComposite)
%shared_ptr(chrono::ChMaterialCompositeNSC)
%shared_ptr(chrono::ChMaterialCompositeSMC)

/* Parse the header file to generate wrappers */
%include "../../chrono/physics/ChMaterialSurface.h"  
%include "../../chrono/physics/ChMaterialSurfaceNSC.h"  
%include "../../chrono/physics/ChMaterialSurfaceSMC.h"  

// DOWNCASTING OF SHARED POINTERS
// 
// This is not automatic in Python + SWIG. To address this issue, we provide the following
// Python-side functions to perform casting manually, using the macro 
// %DefSharedPtrDynamicDowncast(base,derived). 
// Do not specify the "chrono::" namespace before base or derived!
// Later, in python, you can do the following (assuming 'mat' is a ChMaterialSurface)
//    matSMC = chrono.CastToChMaterialSurfaceSMC(mat)
//    matSMC.SetYoungsModulus(1e7)

%DefSharedPtrDynamicDowncast(chrono,ChMaterialSurface,ChMaterialSurfaceNSC)
%DefSharedPtrDynamicDowncast(chrono,ChMaterialSurface,ChMaterialSurfaceSMC)

%DefSharedPtrDynamicDowncast(chrono,ChMaterialComposite,ChMaterialCompositeSMC)
%DefSharedPtrDynamicDowncast(chrono,ChMaterialComposite,ChMaterialCompositeNSC)
