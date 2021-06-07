%{

/* Includes additional C++ in the wrapper code */

#include <string>
#include <vector>
#include "chrono/core/ChVector.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono/physics/ChShaft.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

#include "chrono_vehicle/ChDriveline.h"
#include "chrono_vehicle/wheeled_vehicle/ChDrivelineWV.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDrivelineXWD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ShaftsDriveline2WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/SimpleDriveline.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ShaftsDriveline4WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/SimpleDrivelineXWD.h"

#include "chrono_models/vehicle/generic/Generic_Driveline2WD.h"
#include "chrono_models/vehicle/generic/Generic_SimpleDriveline.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_Driveline2WD.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Driveline4WD.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_SimpleDriveline.h"

#include "chrono_models/vehicle/sedan/Sedan_Driveline2WD.h"

#include "chrono_models/vehicle/citybus/CityBus_Driveline2WD.h"

#include "chrono_models/vehicle/man/MAN_5t_SimpleDriveline.h"
#include "chrono_models/vehicle/man/MAN_5t_SimpleDrivelineXWD.h"

#include "chrono_models/vehicle/uaz/UAZBUS_Driveline4WD.h"
#include "chrono_models/vehicle/uaz/UAZBUS_Driveline2WD.h"

#include "chrono_models/vehicle/gator/Gator_SimpleDriveline.h"
#include "chrono_models/vehicle/gator/Gator_Driveline2WD.h"

#include "chrono_models/vehicle/rccar/RCCar_Driveline4WD.h"

#include "chrono_models/vehicle/feda/FEDA_Driveline4WD.h"

#include "chrono_models/vehicle/m113/M113_SimpleDriveline.h"
#include "chrono_models/vehicle/m113/M113_DrivelineBDS.h"
%}


%shared_ptr(chrono::vehicle::generic::Generic_Driveline2WD)
%shared_ptr(chrono::vehicle::generic::Generic_SimpleDriveline)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Driveline2WD)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_Driveline4WD)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_SimpleDriveline)
%shared_ptr(chrono::vehicle::sedan::Sedan_Driveline2WD)
%shared_ptr(chrono::vehicle::citybus::CityBus_Driveline2WD)
%shared_ptr(chrono::vehicle::man::MAN_5t_SimpleDriveline)
%shared_ptr(chrono::vehicle::man::MAN_5t_SimpleDrivelineXWD)
%shared_ptr(chrono::vehicle::uaz::UAZBUS_Driveline2WD)
%shared_ptr(chrono::vehicle::uaz::UAZBUS_Driveline4WD)
%shared_ptr(chrono::vehicle::gator::Gator_SimpleDriveline)
%shared_ptr(chrono::vehicle::gator::Gator_Driveline2WD)
%shared_ptr(chrono::vehicle::rccar::RCCar_Driveline4WD)
%shared_ptr(chrono::vehicle::feda::FEDA_Driveline4WD)

%shared_ptr(chrono::vehicle::m113::M113_SimpleDriveline)
%shared_ptr(chrono::vehicle::m113::M113_DrivelineBDS)

/* Parse the header file to generate wrappers */
%import "chrono_python/vehicle/ChDriveline.i"

// Model:
%include "../../chrono_models/vehicle/generic/Generic_Driveline2WD.h"
%include "../../chrono_models/vehicle/generic/Generic_SimpleDriveline.h"

%include "../../chrono_models/vehicle/hmmwv/HMMWV_Driveline2WD.h"
%include "../../chrono_models/vehicle/hmmwv/HMMWV_Driveline4WD.h"
%include "../../chrono_models/vehicle/hmmwv/HMMWV_SimpleDriveline.h"

%include "../../chrono_models/vehicle/sedan/Sedan_Driveline2WD.h"

%include "../../chrono_models/vehicle/citybus/CityBus_Driveline2WD.h"

%include "../../chrono_models/vehicle/man/MAN_5t_SimpleDriveline.h"
%include "../../chrono_models/vehicle/man/MAN_5t_SimpleDrivelineXWD.h"

%include "../../chrono_models/vehicle/uaz/UAZBUS_Driveline2WD.h"
%include "../../chrono_models/vehicle/uaz/UAZBUS_Driveline4WD.h"

%include "../../chrono_models/vehicle/gator/Gator_SimpleDriveline.h"
%include "../../chrono_models/vehicle/gator/Gator_Driveline2WD.h"

%include "../../chrono_models/vehicle/rccar/RCCar_Driveline4WD.h"

%include "../../chrono_models/vehicle/feda/FEDA_Driveline4WD.h"

%include "../../chrono_models/vehicle/m113/M113_SimpleDriveline.h"
%include "../../chrono_models/vehicle/m113/M113_DrivelineBDS.h"
