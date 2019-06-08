%{

/* Includes additional C++ in the wrapper code */

#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ReissnerTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ReissnerToroidalTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/LugreTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FEATire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChReissnerTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChPacejkaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChPac2002_data.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChPac89Tire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChLugreTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChFEATire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFToroidalTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"



#include "chrono_thirdparty/rapidjson/document.h"

%}


%shared_ptr(chrono::vehicle::ChTire)
%shared_ptr(chrono::vehicle::ChTMeasyTire)
%shared_ptr(chrono::vehicle::ChRigidTire)
%shared_ptr(chrono::vehicle::ChReissnerTire)
%shared_ptr(chrono::vehicle::ChPacejkaTire)
%shared_ptr(chrono::vehicle::ChPac2002_data)
%shared_ptr(chrono::vehicle::ChPac89Tire)
%shared_ptr(chrono::vehicle::ChLugreTire)
%shared_ptr(chrono::vehicle::ChFialaTire)
%shared_ptr(chrono::vehicle::TMeasyTire)
%shared_ptr(chrono::vehicle::RigidTire)
%shared_ptr(chrono::vehicle::ReissnerToroidalTire)
%shared_ptr(chrono::vehicle::LugreTire)
%shared_ptr(chrono::vehicle::FialaTire)


%import(module = "pychrono.core") "ChShaft.i"
%import "ChTerrain.i"
%import "chrono_vehicle/ChSubsysDefs.h"


/* Parse the header file to generate wrappers */
%include "../chrono_vehicle/wheeled_vehicle/ChTire.h"

%include "../chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"
%ignore chrono::vehicle::ChReissnerTire::ChReissnerTire;
%include "../chrono_vehicle/wheeled_vehicle/tire/ChPacejkaTire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/ChPac2002_data.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/ChPac89Tire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/ChLugreTire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"

%include "../chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/LugreTire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"

/* TODO: wrap deformable models (Not used in ML so far...)
%include "../chrono_vehicle/wheeled_vehicle/tire/ChFEATire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/FEATire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/ChReissnerTire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/ReissnerTire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/ReissnerToroidalTire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/ANCFToroidalTire.h"
%include "../chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"*/

%include "models/TireModels.i"