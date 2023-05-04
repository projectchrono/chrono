%{
#include <string>
#include <vector>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChBody.h"

#include "chrono_vehicle/ChPowertrainAssembly.h"

#include "chrono_vehicle/powertrain/ChEngineSimple.h"
#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"
#include "chrono_vehicle/powertrain/ChEngineShafts.h"
#include "chrono_vehicle/powertrain/EngineSimple.h"
#include "chrono_vehicle/powertrain/EngineSimpleMap.h"
#include "chrono_vehicle/powertrain/EngineShafts.h"

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionShafts.h"
#include "chrono_vehicle/powertrain/AutomaticTransmissionSimpleMap.h"
#include "chrono_vehicle/powertrain/AutomaticTransmissionShafts.h"
%}

%shared_ptr(chrono::vehicle::ChPowertrainAssembly)

%shared_ptr(chrono::vehicle::ChEngineSimple)
%shared_ptr(chrono::vehicle::ChEngineSimpleMap)
%shared_ptr(chrono::vehicle::ChEngineShafts)
%shared_ptr(chrono::vehicle::EngineSimple)
%shared_ptr(chrono::vehicle::EngineSimpleMap)
%shared_ptr(chrono::vehicle::EngineShafts)

%shared_ptr(chrono::vehicle::ChAutomaticTransmissionSimpleMap)
%shared_ptr(chrono::vehicle::ChAutomaticTransmissionShafts)
%shared_ptr(chrono::vehicle::AutomaticTransmissionSimpleMap)
%shared_ptr(chrono::vehicle::AutomaticTransmissionShafts)


%import "../../../chrono_vehicle/ChPart.h"

#ifdef SWIGCSHARP
%import "chrono_swig/interface/core/ChShaft.i"
#endif

#ifdef SWIGPYCHRONO
%import(module = "pychrono.core") "chrono_swig/interface/core/ChShaft.i"
#endif

// Parse the header file to generate wrappers
%include "../../../chrono_vehicle/ChPowertrainAssembly.h"

%include "../../../chrono_vehicle/powertrain/ChEngineSimple.h"
%include "../../../chrono_vehicle/powertrain/ChEngineSimpleMap.h"
%include "../../../chrono_vehicle/powertrain/ChEngineShafts.h"
%include "../../../chrono_vehicle/powertrain/EngineSimple.h"
%include "../../../chrono_vehicle/powertrain/EngineSimpleMap.h"
%include "../../../chrono_vehicle/powertrain/EngineShafts.h"

%include "../../../chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"
%include "../../../chrono_vehicle/powertrain/ChAutomaticTransmissionShafts.h"
%include "../../../chrono_vehicle/powertrain/AutomaticTransmissionSimpleMap.h"
%include "../../../chrono_vehicle/powertrain/AutomaticTransmissionShafts.h"

%include "chrono_swig/interface/models/PowertrainModels.i"
