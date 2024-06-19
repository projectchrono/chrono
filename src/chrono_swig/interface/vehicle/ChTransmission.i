#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

%csmethodmodifiers chrono::vehicle::ChTransmission::GetType "public virtual new"

#endif             // ----------------------------------------------------------------- end CSHARP

%{
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/ChTransmission.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"
#include "chrono_vehicle/powertrain/AutomaticTransmissionSimpleMap.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionShafts.h"
#include "chrono_vehicle/powertrain/AutomaticTransmissionShafts.h"
#include "chrono_vehicle/powertrain/ChManualTransmissionShafts.h"
#include "chrono_vehicle/powertrain/ManualTransmissionShafts.h"
%}

%shared_ptr(chrono::vehicle::ChAutomaticTransmission)
%shared_ptr(chrono::vehicle::ChManualTransmission) 
%shared_ptr(chrono::vehicle::ChAutomaticTransmissionSimpleMap)
%shared_ptr(chrono::vehicle::AutomaticTransmissionSimpleMap)
%shared_ptr(chrono::vehicle::ChAutomaticTransmissionShafts)
%shared_ptr(chrono::vehicle::AutomaticTransmissionShafts)
%shared_ptr(chrono::vehicle::ChManualTransmissionShafts)
%shared_ptr(chrono::vehicle::ManualTransmissionShafts)  

%include "../../../chrono_vehicle/ChPart.h"
%include "../../../chrono_vehicle/ChTransmission.h"
%include "../../../chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"
%include "../../../chrono_vehicle/powertrain/AutomaticTransmissionSimpleMap.h"
%include "../../../chrono_vehicle/powertrain/ChAutomaticTransmissionShafts.h"
%include "../../../chrono_vehicle/powertrain/AutomaticTransmissionShafts.h"
%include "../../../chrono_vehicle/powertrain/ChManualTransmissionShafts.h"
%include "../../../chrono_vehicle/powertrain/ManualTransmissionShafts.h"

