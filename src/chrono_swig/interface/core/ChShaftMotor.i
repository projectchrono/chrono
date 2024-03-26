%{

/* Includes the header in the wrapper code */

#include "chrono/physics/ChShaftsMotorPosition.h"
#include "chrono/physics/ChShaftsMotorSpeed.h"
#include "chrono/physics/ChShaftsMotorLoad.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChSystem.i"
%import "ChPhysicsItem.i"
%import "ChShaft.i"


%shared_ptr(chrono::ChShaftsMotorPosition)
%shared_ptr(chrono::ChShaftsMotorSpeed)
%shared_ptr(chrono::ChShaftsMotorLoad)


/* Parse the header file to generate wrappers */

%include "../../../chrono/physics/ChShaftsMotorPosition.h"
%include "../../../chrono/physics/ChShaftsMotorSpeed.h"
%include "../../../chrono/physics/ChShaftsMotorLoad.h"
