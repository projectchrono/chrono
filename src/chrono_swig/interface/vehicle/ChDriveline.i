%{

/* Includes additional C++ in the wrapper code */

#include <string>
#include <vector>

#include "chrono/core/ChVector.h"
#include "chrono/assets/ChColor.h"
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

#include "chrono_vehicle/tracked_vehicle/ChDrivelineTV.h"
#include "chrono_vehicle/tracked_vehicle/driveline/ChSimpleTrackDriveline.h"
#include "chrono_vehicle/tracked_vehicle/driveline/ChTrackDrivelineBDS.h"
#include "chrono_vehicle/tracked_vehicle/driveline/SimpleTrackDriveline.h"
#include "chrono_vehicle/tracked_vehicle/driveline/TrackDrivelineBDS.h"

#include "chrono_thirdparty/rapidjson/document.h"

%}

%shared_ptr(chrono::vehicle::ChDriveline)

%shared_ptr(chrono::vehicle::ChDrivelineWV)
%shared_ptr(chrono::vehicle::ChShaftsDriveline2WD)
%shared_ptr(chrono::vehicle::ChShaftsDriveline4WD)
%shared_ptr(chrono::vehicle::ChSimpleDriveline)
%shared_ptr(chrono::vehicle::ChSimpleDrivelineXWD)
%shared_ptr(chrono::vehicle::ShaftsDriveline2WD)
%shared_ptr(chrono::vehicle::SimpleDriveline)
%shared_ptr(chrono::vehicle::ShaftsDriveline4WD)
%shared_ptr(chrono::vehicle::SimpleDrivelineXWD)

%shared_ptr(chrono::vehicle::ChDrivelineTV)
%shared_ptr(chrono::vehicle::ChSimpleTrackDriveline)
%shared_ptr(chrono::vehicle::ChTrackDrivelineBDS)
%shared_ptr(chrono::vehicle::SimpleTrackDriveline)
%shared_ptr(chrono::vehicle::TrackDrivelineBDS)

#ifdef SWIGCSHARP
%import "chrono_swig/interface/core/ChShaft.i"
#endif

#ifdef SWIGPYCHRONO
%import(module = "pychrono.core") "chrono_swig/interface/core/ChShaft.i"
#endif

/* Parse the header file to generate wrappers */
%include "../../../chrono_vehicle/ChDriveline.h"

%include "../../../chrono_vehicle/wheeled_vehicle/ChDrivelineWV.h"
%include "../../../chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"
%include "../../../chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"
%include "../../../chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline.h"
%include "../../../chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDrivelineXWD.h"
%include "../../../chrono_vehicle/wheeled_vehicle/driveline/ShaftsDriveline2WD.h"
%include "../../../chrono_vehicle/wheeled_vehicle/driveline/SimpleDriveline.h"
%include "../../../chrono_vehicle/wheeled_vehicle/driveline/ShaftsDriveline4WD.h"
%include "../../../chrono_vehicle/wheeled_vehicle/driveline/SimpleDrivelineXWD.h"

%include "../../../chrono_vehicle/tracked_vehicle/ChDrivelineTV.h"
%include "../../../chrono_vehicle/tracked_vehicle/driveline/ChSimpleTrackDriveline.h"
%include "../../../chrono_vehicle/tracked_vehicle/driveline/ChTrackDrivelineBDS.h"
%include "../../../chrono_vehicle/tracked_vehicle/driveline/SimpleTrackDriveline.h"
%include "../../../chrono_vehicle/tracked_vehicle/driveline/TrackDrivelineBDS.h"

%include "chrono_swig/interface/models/DrivelineModels.i"
