%{

/* Includes the header in the wrapper code */
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




/* Parse the header file to generate wrappers */
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






