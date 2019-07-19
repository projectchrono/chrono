%{

/* Includes the header in the wrapper code */
#include "chrono/solver/ChVariables.h"
#include "chrono/solver/ChVariablesShaft.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/physics/ChShaftsTorqueBase.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftsGear.h"
#include "chrono/physics/ChShaftsMotor.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/physics/ChShaftsThermalEngine.h"
#include "chrono/physics/ChShaftsTorsionSpring.h"
#include "chrono/physics/ChShaftsLoads.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChSystem.i"
%import "ChPhysicsItem.i"
%import "ChLoad.i"
%import "ChLoadable.i"

%shared_ptr(chrono::ChVariables)
%shared_ptr(chrono::ChVariablesShaft)
%shared_ptr(chrono::ChShaft)
%shared_ptr(chrono::ChShaftsCouple)
%shared_ptr(chrono::ChShaftsBody)
%shared_ptr(chrono::ChShaftsBodyTranslation)
%shared_ptr(chrono::ChShaftsMotorBase)
%shared_ptr(chrono::ChShaftsClutch)
%shared_ptr(chrono::ChShaftsGear)
%shared_ptr(chrono::ChShaftsMotor)
%shared_ptr(chrono::ChShaftsPlanetary)
%shared_ptr(chrono::ChShaftsThermalEngine)
%shared_ptr(chrono::ChShaftsTorqueBase)
%shared_ptr(chrono::ChShaftsTorsionSpring)
%shared_ptr(chrono::ChShaftsLoad)
%shared_ptr(chrono::ChShaftsTorsionSpringDamper)
%shared_ptr(chrono::ChShaftsElasticGear)


/* Parse the header file to generate wrappers */
%include "../chrono/solver/ChVariables.h"
%include "../chrono/solver/ChVariablesShaft.h"
%include "../chrono/physics/ChShaft.h"  
%include "../chrono/physics/ChShaftsBody.h" 
%include "../chrono/physics/ChShaftsCouple.h" 
%include "../chrono/physics/ChShaftsTorqueBase.h" 
%include "../chrono/physics/ChShaftsPlanetary.h"  
%include "../chrono/physics/ChShaftsGear.h"
%include "../chrono/physics/ChShaftsMotor.h"  
%include "../chrono/physics/ChShaftsClutch.h"  
%include "../chrono/physics/ChShaftsThermalEngine.h"  
%include "../chrono/physics/ChShaftsTorsionSpring.h"  
%include "../chrono/physics/ChShaftsLoads.h"  






