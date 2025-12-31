%{
#include <string>
#include <vector>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/assets/ChVisualShapeCylinder.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChBody.h"

#include "chrono_vehicle/ChPowertrainAssembly.h"

#include "chrono_vehicle/powertrain/ChEngineSimple.h"
#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"
#include "chrono_vehicle/powertrain/ChEngineShafts.h"
#include "chrono_vehicle/powertrain/EngineSimple.h"
#include "chrono_vehicle/powertrain/EngineSimpleMap.h"
#include "chrono_vehicle/powertrain/EngineShafts.h"

#include "chrono_vehicle/ChTransmission.h"

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionShafts.h"
#include "chrono_vehicle/powertrain/AutomaticTransmissionSimpleMap.h"
#include "chrono_vehicle/powertrain/AutomaticTransmissionShafts.h"
#include "chrono_vehicle/powertrain/ChManualTransmissionShafts.h"
#include "chrono_vehicle/powertrain/ManualTransmissionShafts.h"
%}

%shared_ptr(chrono::vehicle::ChPowertrainAssembly)

%shared_ptr(chrono::vehicle::ChEngineSimple)
%shared_ptr(chrono::vehicle::ChEngineSimpleMap)
%shared_ptr(chrono::vehicle::ChEngineShafts)
%shared_ptr(chrono::vehicle::EngineSimple)
%shared_ptr(chrono::vehicle::EngineSimpleMap)
%shared_ptr(chrono::vehicle::EngineShafts)

%import "../../../chrono_vehicle/ChPart.h"

#ifdef SWIGCSHARP
%import "chrono_swig/interface/core/ChShaft.i"
#endif

#ifdef SWIGPYCHRONO
%import(module = "pychrono.core") "chrono_swig/interface/core/ChShaft.i"
#endif

// Parse the header file to generate wrappers
%include "../../../chrono_vehicle/ChPowertrainAssembly.h"

#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP
// Mark override methods to ensure swig knows how to handle overides of base classes (for Unity)
%csmethodmodifiers chrono::vehicle::ChEngineSimple::GetTemplateName "public override"
%csmethodmodifiers chrono::vehicle::ChEngineSimple::GetMotorSpeed "public override"
%csmethodmodifiers chrono::vehicle::ChEngineSimple::GetOutputMotorshaftTorque "public override"

%csmethodmodifiers chrono::vehicle::ChEngineSimpleMap::GetTemplateName "public override"
%csmethodmodifiers chrono::vehicle::ChEngineSimpleMap::GetMotorSpeed "public override"
%csmethodmodifiers chrono::vehicle::ChEngineSimpleMap::GetOutputMotorshaftTorque "public override"

%csmethodmodifiers chrono::vehicle::ChEngineShafts::GetTemplateName "public override"
%csmethodmodifiers chrono::vehicle::ChEngineShafts::GetMotorSpeed "public override"
%csmethodmodifiers chrono::vehicle::ChEngineShafts::GetOutputMotorshaftTorque "public override"
%csmethodmodifiers chrono::vehicle::ChEngineShafts::GetChassisReactionTorque "public override"
#endif             // --------------------------------------------------------------------- CSHARP

%include "../../../chrono_vehicle/powertrain/ChEngineSimple.h"
%include "../../../chrono_vehicle/powertrain/ChEngineSimpleMap.h"
%include "../../../chrono_vehicle/powertrain/ChEngineShafts.h"
%include "../../../chrono_vehicle/powertrain/EngineSimple.h"
%include "../../../chrono_vehicle/powertrain/EngineSimpleMap.h"
%include "../../../chrono_vehicle/powertrain/EngineShafts.h"

