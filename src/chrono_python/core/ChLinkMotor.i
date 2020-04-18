%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkMotor.h"

#include "chrono/physics/ChLinkMotorLinear.h"
#include "chrono/physics/ChLinkMotorLinearDriveline.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"

#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationDriveline.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLinkMate.i"


/* Parse the header file to generate wrappers */
%include "../../chrono/physics/ChLinkMotor.h" 
%include "../../chrono/physics/ChLinkMotorLinear.h" 
%include "../../chrono/physics/ChLinkMotorLinearDriveline.h" 
%include "../../chrono/physics/ChLinkMotorLinearForce.h" 
%include "../../chrono/physics/ChLinkMotorLinearPosition.h" 
%include "../../chrono/physics/ChLinkMotorLinearSpeed.h" 
%include "../../chrono/physics/ChLinkMotorRotation.h" 
%include "../../chrono/physics/ChLinkMotorRotationAngle.h" 
%include "../../chrono/physics/ChLinkMotorRotationDriveline.h" 
%include "../../chrono/physics/ChLinkMotorRotationSpeed.h" 
%include "../../chrono/physics/ChLinkMotorRotationTorque.h"  







