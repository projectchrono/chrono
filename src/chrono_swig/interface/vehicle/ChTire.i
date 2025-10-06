%{
#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChFrame.h"
#include "chrono/functions/ChFunctionInterp.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChTMsimpleTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChPac89Tire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChPac02Tire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMsimpleTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac89Tire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac02Tire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChFEATire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChReissnerTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FEATire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFToroidalTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ReissnerTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ReissnerToroidalTire.h"

#include "chrono_thirdparty/rapidjson/document.h"
%}

%shared_ptr(chrono::vehicle::ChTire)

%shared_ptr(chrono::vehicle::ChRigidTire)
%shared_ptr(chrono::vehicle::RigidTire)

%shared_ptr(chrono::vehicle::ChForceElementTire)
%shared_ptr(chrono::vehicle::ChTMeasyTire)
%shared_ptr(chrono::vehicle::ChTMsimpleTire)
%shared_ptr(chrono::vehicle::ChPac89Tire)
%shared_ptr(chrono::vehicle::ChPac02Tire)
%shared_ptr(chrono::vehicle::ChFialaTire)
%shared_ptr(chrono::vehicle::TMeasyTire)
%shared_ptr(chrono::vehicle::TMsimpleTire)
%shared_ptr(chrono::vehicle::Pac89Tire)
%shared_ptr(chrono::vehicle::Pac02Tire)
%shared_ptr(chrono::vehicle::FialaTire)

%shared_ptr(chrono::vehicle::ChDeformableTire)
%shared_ptr(chrono::vehicle::ChFEATire)
%shared_ptr(chrono::vehicle::ChANCFTire)
%shared_ptr(chrono::vehicle::ChReissnerTire)
%shared_ptr(chrono::vehicle::FEATire)
%shared_ptr(chrono::vehicle::ANCFTire)
%shared_ptr(chrono::vehicle::ANCFToroidalTire)
%shared_ptr(chrono::vehicle::ReissnerTire)
%shared_ptr(chrono::vehicle::ReissnerToroidalTire)

#ifdef SWIGCSHARP
%import "chrono_swig/interface/core/ChShaft.i"
#endif

#ifdef SWIGPYCHRONO
%import(module = "pychrono.core") "chrono_swig/interface/core/ChShaft.i"
#endif

%import "ChTerrain.i"
%import "chrono_vehicle/ChSubsysDefs.h"

// Parse the header file to generate wrappers
%include "../../../chrono_vehicle/wheeled_vehicle/ChTire.h"

%include "../../../chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"

%include "../../../chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/ChTMsimpleTire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/ChPac89Tire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/ChPac02Tire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/TMsimpleTire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/Pac89Tire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/Pac02Tire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"

%include "../../../chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/ChFEATire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/ChReissnerTire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/FEATire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/ANCFToroidalTire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/ReissnerTire.h"
%include "../../../chrono_vehicle/wheeled_vehicle/tire/ReissnerToroidalTire.h"

%include "chrono_swig/interface/models/TireModels.i"
