#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// MULTIPLE INHERITANCE WORKAROUND

// (B) Methods of a base class that SWIG discards that *are* overriden in ChShaft

// Ensure that these functions are not marked as 'overrides' in the generated C# code.

%csmethodmodifiers chrono::ChShaft::GetLoadableNumCoordsPosLevel "public"
%csmethodmodifiers chrono::ChShaft::GetLoadableNumCoordsVelLevel "public"
%csmethodmodifiers chrono::ChShaft::LoadableGetStateBlockPosLevel "public"
%csmethodmodifiers chrono::ChShaft::LoadableGetStateBlockVelLevel "public"
%csmethodmodifiers chrono::ChShaft::LoadableStateIncrement "public"
%csmethodmodifiers chrono::ChShaft::GetNumFieldCoords "public"
%csmethodmodifiers chrono::ChShaft::GetNumSubBlocks "public"
%csmethodmodifiers chrono::ChShaft::GetSubBlockOffset "public"
%csmethodmodifiers chrono::ChShaft::GetSubBlockSize "public"
%csmethodmodifiers chrono::ChShaft::IsSubBlockActive "public"
%csmethodmodifiers chrono::ChShaft::LoadableGetVariables "public"

//// RADU:  Do we actually want to wrap methods of ChLoadable?
////        If not, we should probably just use %ignore

#endif             // --------------------------------------------------------------------- CSHARP

%{
//#include "chrono/solver/ChVariables.h"
//#include "chrono/solver/ChVariablesShaft.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/physics/ChShaftsTorque.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftsGear.h"
#include "chrono/physics/ChShaftsMotor.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/physics/ChShaftsThermalEngine.h"
#include "chrono/physics/ChShaftsTorsionSpring.h"
#include "chrono/physics/ChShaftsAppliedTorque.h"
#include "chrono/physics/ChShaftsLoads.h"
#include "chrono/physics/ChShaftsFreewheel.h"
%}
 
// Tell SWIG about parent classes
%import "chrono_swig/interface/core/ChSystem.i"
%import "chrono_swig/interface/core/ChPhysicsItem.i"
%import "chrono_swig/interface/core/ChLoad.i"
%import "chrono_swig/interface/core/ChLoadable.i"

//%shared_ptr(chrono::ChVariables)
//%shared_ptr(chrono::ChVariablesShaft)
%shared_ptr(chrono::ChShaft)
%shared_ptr(chrono::ChShaftBodyRotation)
%shared_ptr(chrono::ChShaftBodyTranslation)
%shared_ptr(chrono::ChShaftsCouple)
%shared_ptr(chrono::ChShaftBodyTranslation)
%shared_ptr(chrono::ChShaftsClutch)
%shared_ptr(chrono::ChShaftsGear)
%shared_ptr(chrono::ChShaftsMotor)
%shared_ptr(chrono::ChShaftsPlanetary)
%shared_ptr(chrono::ChShaftsThermalEngine)
%shared_ptr(chrono::ChShaftsTorque)
%shared_ptr(chrono::ChShaftsTorsionSpring)
%shared_ptr(chrono::ChShaftsAppliedTorque)
%shared_ptr(chrono::ChShaftsLoad)
%shared_ptr(chrono::ChShaftsTorsionSpringDamper)
%shared_ptr(chrono::ChShaftsElasticGear)
%shared_ptr(chrono::ChShaftsFreewheel)

// Parse the header file to generate wrappers
//%include "../../../chrono/solver/ChVariables.h"
//%include "../../../chrono/solver/ChVariablesShaft.h"
%include "../../../chrono/physics/ChShaft.h"
%include "../../../chrono/physics/ChShaftBodyConstraint.h"
%include "../../../chrono/physics/ChShaftsCouple.h"
%include "../../../chrono/physics/ChShaftsTorque.h"
%include "../../../chrono/physics/ChShaftsPlanetary.h"
%include "../../../chrono/physics/ChShaftsGear.h"
%include "../../../chrono/physics/ChShaftsMotor.h"
%include "../../../chrono/physics/ChShaftsClutch.h"
%include "../../../chrono/physics/ChShaftsThermalEngine.h"  
%include "../../../chrono/physics/ChShaftsTorsionSpring.h"
%include "../../../chrono/physics/ChShaftsAppliedTorque.h"
%include "../../../chrono/physics/ChShaftsLoads.h"
%include "../../../chrono/physics/ChShaftsFreewheel.h"






