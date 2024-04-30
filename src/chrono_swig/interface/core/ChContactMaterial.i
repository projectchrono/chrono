%{

/* Includes the header in the wrapper code */

#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChContactMaterialSMC.h"
using namespace chrono;

%};

%shared_ptr(chrono::ChContactMaterial)
%shared_ptr(chrono::ChContactMaterialNSC)
%shared_ptr(chrono::ChContactMaterialSMC)
%shared_ptr(chrono::ChContactMaterialComposite)
%shared_ptr(chrono::ChContactMaterialCompositeNSC)
%shared_ptr(chrono::ChContactMaterialCompositeSMC)

/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChContactMaterial.h"  
%include "../../../chrono/physics/ChContactMaterialNSC.h"  
%include "../../../chrono/physics/ChContactMaterialSMC.h"  

// CASTING OF SHARED POINTERS
// 
// This is not automatic in Python + SWIG. To address this issue, we provide the following
// Python-side functions to perform casting manually, using the macro 
// %DefSharedPtrDynamicCast(base,derived). 
// Do not specify the "chrono::" namespace before base or derived!
// Later, in python, you can do the following (assuming 'mat' is a ChContactMaterial)
//    matSMC = chrono.CastToChContactMaterialSMC(mat)
//    matSMC.SetYoungsModulus(1e7)

%DefSharedPtrDynamicCast(chrono,ChContactMaterial,ChContactMaterialNSC)
%DefSharedPtrDynamicCast(chrono,ChContactMaterial,ChContactMaterialSMC)

%DefSharedPtrDynamicCast(chrono,ChContactMaterialComposite,ChContactMaterialCompositeSMC)
%DefSharedPtrDynamicCast(chrono,ChContactMaterialComposite,ChContactMaterialCompositeNSC)
