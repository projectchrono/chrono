#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

%csmethodmodifiers chrono::vehicle::ChTransmission::GetType "public virtual new"

#endif             // --------------------------------------------------------------------- CSHARP


%{
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/ChTransmission.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"  // Included for testing
%}


%include "chrono_vehicle/ChPart.h"
%include "chrono_vehicle/ChTransmission.h"
// Include ChPowertrainAssembly if necessary
%include "chrono_vehicle/ChPowertrainAssembly.h"

