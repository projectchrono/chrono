#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// MULTIPLE INHERITANCE WORKAROUND

// Extend ChLinkMotor with Initialize functions that take two ChBody (not ChBodyFrame)
%extend chrono::ChLinkMotor
{
  void Initialize(std::shared_ptr<ChBody> body1, std::shared_ptr<ChBody> body2, ChFrame<double> absframe) {
     $self->Initialize(std::dynamic_pointer_cast<ChBodyFrame>(body1), std::dynamic_pointer_cast<ChBodyFrame>(body2), absframe);
  }
}

#endif             // --------------------------------------------------------------------- CSHARP

%{
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


// Parse the header file to generate wrappers
%include "../../../chrono/physics/ChLinkMotor.h" 
%include "../../../chrono/physics/ChLinkMotorLinear.h" 
%include "../../../chrono/physics/ChLinkMotorLinearDriveline.h" 
%include "../../../chrono/physics/ChLinkMotorLinearForce.h" 
%include "../../../chrono/physics/ChLinkMotorLinearPosition.h" 
%include "../../../chrono/physics/ChLinkMotorLinearSpeed.h" 
%include "../../../chrono/physics/ChLinkMotorRotation.h" 
%include "../../../chrono/physics/ChLinkMotorRotationAngle.h" 
%include "../../../chrono/physics/ChLinkMotorRotationDriveline.h" 
%include "../../../chrono/physics/ChLinkMotorRotationSpeed.h" 
%include "../../../chrono/physics/ChLinkMotorRotationTorque.h"  







