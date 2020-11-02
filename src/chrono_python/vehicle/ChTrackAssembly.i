%{

/* Includes additional C++ in the wrapper code */

#include <string>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketSinglePin.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketBand.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/SprocketSinglePin.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/SprocketDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/SprocketBand.h"

#include "chrono_vehicle/tracked_vehicle/ChIdler.h"
#include "chrono_vehicle/tracked_vehicle/idler/ChSingleIdler.h"
#include "chrono_vehicle/tracked_vehicle/idler/ChDoubleIdler.h"
#include "chrono_vehicle/tracked_vehicle/idler/SingleIdler.h"
#include "chrono_vehicle/tracked_vehicle/idler/DoubleIdler.h"

#include "chrono_vehicle/tracked_vehicle/ChRoadWheel.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/ChSingleRoadWheel.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/ChDoubleRoadWheel.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/SingleRoadWheel.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/DoubleRoadWheel.h"

#include "chrono_vehicle/tracked_vehicle/ChRoadWheelAssembly.h"
#include "chrono_vehicle/tracked_vehicle/suspension/ChLinearDamperRWAssembly.h"
#include "chrono_vehicle/tracked_vehicle/suspension/ChRotationalDamperRWAssembly.h"
#include "chrono_vehicle/tracked_vehicle/suspension/LinearDamperRWAssembly.h"
#include "chrono_vehicle/tracked_vehicle/suspension/RotationalDamperRWAssembly.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBand.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandANCF.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandBushing.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSegmented.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSinglePin.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeBandANCF.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeBandBushing.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeSinglePin.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeDoublePin.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBand.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBandANCF.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBandBushing.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySegmented.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySinglePin.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyBandANCF.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyBandBushing.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblySinglePin.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyDoublePin.h"


#include "chrono_thirdparty/rapidjson/document.h"
%}

%shared_ptr(chrono::vehicle::ChSprocket)
%shared_ptr(chrono::vehicle::ChSprocketSinglePin)
%shared_ptr(chrono::vehicle::ChSprocketDoublePin)
%shared_ptr(chrono::vehicle::ChSprocketBand)
%shared_ptr(chrono::vehicle::SprocketSinglePin)
%shared_ptr(chrono::vehicle::SprocketDoublePin)
%shared_ptr(chrono::vehicle::SprocketBand)

%shared_ptr(chrono::vehicle::ChIdler)
%shared_ptr(chrono::vehicle::ChSingleIdler)
%shared_ptr(chrono::vehicle::ChDoubleIdler)
%shared_ptr(chrono::vehicle::SingleIdler)
%shared_ptr(chrono::vehicle::DoubleIdler)

%shared_ptr(chrono::vehicle::ChRoadWheel)
%shared_ptr(chrono::vehicle::ChSingleRoadWheel)
%shared_ptr(chrono::vehicle::ChDoubleRoadWheel)
%shared_ptr(chrono::vehicle::SingleRoadWheel)
%shared_ptr(chrono::vehicle::DoubleRoadWheel)

%shared_ptr(chrono::vehicle::ChRoadWheelAssembly)
%shared_ptr(chrono::vehicle::ChLinearDamperRWAssembly)
%shared_ptr(chrono::vehicle::ChRotationalDamperRWAssembly)
%shared_ptr(chrono::vehicle::LinearDamperRWAssembly)
%shared_ptr(chrono::vehicle::RotationalDamperRWAssembly)

%shared_ptr(chrono::vehicle::ChTrackShoe)
%shared_ptr(chrono::vehicle::ChTrackShoeBand)
%shared_ptr(chrono::vehicle::ChTrackShoeBandANCF)
%shared_ptr(chrono::vehicle::ChTrackShoeBandBushing)
%shared_ptr(chrono::vehicle::ChTrackShoeSegmented)
%shared_ptr(chrono::vehicle::ChTrackShoeSinglePin)
%shared_ptr(chrono::vehicle::ChTrackShoeDoublePin)
%shared_ptr(chrono::vehicle::TrackShoeBandANCF)
%shared_ptr(chrono::vehicle::TrackShoeBandBushing)
%shared_ptr(chrono::vehicle::TrackShoeSinglePin)
%shared_ptr(chrono::vehicle::TrackShoeDoublePin)

%shared_ptr(chrono::vehicle::ChTrackAssembly)
%shared_ptr(chrono::vehicle::ChTrackAssemblyBand)
%shared_ptr(chrono::vehicle::ChTrackAssemblyBandANCF)
%shared_ptr(chrono::vehicle::ChTrackAssemblyBandBushing)
%shared_ptr(chrono::vehicle::ChTrackAssemblySegmented)
%shared_ptr(chrono::vehicle::ChTrackAssemblySinglePin)
%shared_ptr(chrono::vehicle::ChTrackAssemblyDoublePin)
%shared_ptr(chrono::vehicle::TrackAssemblyBandANCF)
%shared_ptr(chrono::vehicle::TrackAssemblyBandBushing)
%shared_ptr(chrono::vehicle::TrackAssemblySinglePin)
%shared_ptr(chrono::vehicle::TrackAssemblyDoublePin)

%import(module = "pychrono.core") "chrono_python/core/ChShaft.i"
%import "../../chrono_vehicle/ChPart.h"

/* Parse the header file to generate wrappers */

%include "../../chrono_vehicle/tracked_vehicle/ChSprocket.h"
%include "../../chrono_vehicle/tracked_vehicle/sprocket/ChSprocketSinglePin.h"
%include "../../chrono_vehicle/tracked_vehicle/sprocket/ChSprocketDoublePin.h"
%include "../../chrono_vehicle/tracked_vehicle/sprocket/ChSprocketBand.h"
%include "../../chrono_vehicle/tracked_vehicle/sprocket/SprocketSinglePin.h"
%include "../../chrono_vehicle/tracked_vehicle/sprocket/SprocketDoublePin.h"
%include "../../chrono_vehicle/tracked_vehicle/sprocket/SprocketBand.h"

%ignore chrono::vehicle::ChIdler::GetLocation;
%include "../../chrono_vehicle/tracked_vehicle/ChIdler.h"
%include "../../chrono_vehicle/tracked_vehicle/idler/ChSingleIdler.h"
%include "../../chrono_vehicle/tracked_vehicle/idler/ChDoubleIdler.h"
%include "../../chrono_vehicle/tracked_vehicle/idler/SingleIdler.h"
%include "../../chrono_vehicle/tracked_vehicle/idler/DoubleIdler.h"

%include "../../chrono_vehicle/tracked_vehicle/ChRoadWheel.h"
%include "../../chrono_vehicle/tracked_vehicle/road_wheel/ChSingleRoadWheel.h"
%include "../../chrono_vehicle/tracked_vehicle/road_wheel/ChDoubleRoadWheel.h"
%include "../../chrono_vehicle/tracked_vehicle/road_wheel/SingleRoadWheel.h"
%include "../../chrono_vehicle/tracked_vehicle/road_wheel/DoubleRoadWheel.h"

%ignore chrono::vehicle::ChLinearDamperRWAssembly::GetLocation;
%include "../../chrono_vehicle/tracked_vehicle/ChRoadWheelAssembly.h"
%include "../../chrono_vehicle/tracked_vehicle/suspension/ChLinearDamperRWAssembly.h"
%include "../../chrono_vehicle/tracked_vehicle/suspension/ChRotationalDamperRWAssembly.h"
%include "../../chrono_vehicle/tracked_vehicle/suspension/LinearDamperRWAssembly.h"
%include "../../chrono_vehicle/tracked_vehicle/suspension/RotationalDamperRWAssembly.h"

%include "../../chrono_vehicle/tracked_vehicle/ChTrackShoe.h"
%include "../../chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBand.h"
%include "../../chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandANCF.h"
%include "../../chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandBushing.h"
%include "../../chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSegmented.h"
%include "../../chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSinglePin.h"
%include "../../chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"
%include "../../chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeBandANCF.h"
%include "../../chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeBandBushing.h"
%include "../../chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeSinglePin.h"
%include "../../chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeDoublePin.h"

%include "../../chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
%include "../../chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBand.h"
%include "../../chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBandANCF.h"
%include "../../chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBandBushing.h"
%include "../../chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySegmented.h"
%include "../../chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySinglePin.h"
%include "../../chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyDoublePin.h"
%include "../../chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyBandANCF.h"
%include "../../chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyBandBushing.h"
%include "../../chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblySinglePin.h"
%include "../../chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyDoublePin.h"


%include "chrono_python/models/TrackAssemblyModels.i"
