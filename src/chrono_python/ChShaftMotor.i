%{

/* Includes the header in the wrapper code */

#include "chrono/physics/ChShaftsMotorAngle.h"
#include "chrono/physics/ChShaftsMotorSpeed.h"
#include "chrono/physics/ChShaftsMotorTorque.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChSystem.i"
%import "ChPhysicsItem.i"
%import "ChShaft.i"


%shared_ptr(chrono::ChShaftsMotorAngle)
%shared_ptr(chrono::ChShaftsMotorSpeed)
%shared_ptr(chrono::ChShaftsMotorTorque)


/* Parse the header file to generate wrappers */

%include "../chrono/physics/ChShaftsMotorAngle.h"
%include "../chrono/physics/ChShaftsMotorSpeed.h"
%include "../chrono/physics/ChShaftsMotorTorque.h"