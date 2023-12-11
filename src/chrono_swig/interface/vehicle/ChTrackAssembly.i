#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP
%csmethodmodifiers chrono::vehicle::ChTrackWheel::GetType "public virtual new"
%csmethodmodifiers chrono::vehicle::ChTrackSuspension::GetType "public virtual new"
%csmethodmodifiers chrono::vehicle::ChTrackShoe::GetType "public virtual new"
%csmethodmodifiers chrono::vehicle::ChIdler::GetType "public virtual new"
%csmethodmodifiers chrono::vehicle::ChTrackWheel::GetInertia "public virtual new"
#endif             // --------------------------------------------------------------------- CSHARP

%{
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
#include "chrono_vehicle/tracked_vehicle/idler/ChTranslationalIdler.h"
//#include "chrono_vehicle/tracked_vehicle/idler/ChDistanceIdler.h"
#include "chrono_vehicle/tracked_vehicle/idler/TranslationalIdler.h"
//#include "chrono_vehicle/tracked_vehicle/idler/DistanceIdler.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackWheel.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/ChSingleTrackWheel.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/ChDoubleTrackWheel.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/SingleTrackWheel.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/DoubleTrackWheel.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackSuspension.h"
#include "chrono_vehicle/tracked_vehicle/suspension/ChTranslationalDamperSuspension.h"
#include "chrono_vehicle/tracked_vehicle/suspension/ChRotationalDamperSuspension.h"
#include "chrono_vehicle/tracked_vehicle/suspension/TranslationalDamperSuspension.h"
#include "chrono_vehicle/tracked_vehicle/suspension/RotationalDamperSuspension.h"

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
%shared_ptr(chrono::vehicle::ChTranslationalIdler)
//%shared_ptr(chrono::vehicle::ChDistanceIdler)
%shared_ptr(chrono::vehicle::TranslationalIdler)
//%shared_ptr(chrono::vehicle::DistanceIdler)

%shared_ptr(chrono::vehicle::ChTrackWheel)
%shared_ptr(chrono::vehicle::ChSingleTrackWheel)
%shared_ptr(chrono::vehicle::ChDoubleTrackWheel)
%shared_ptr(chrono::vehicle::SingleTrackWheel)
%shared_ptr(chrono::vehicle::DoubleTrackWheel)

%shared_ptr(chrono::vehicle::ChTrackSuspension)
%shared_ptr(chrono::vehicle::ChTranslationalDamperSuspension)
%shared_ptr(chrono::vehicle::ChRotationalDamperSuspension)
%shared_ptr(chrono::vehicle::TranslationalDamperSuspension)
%shared_ptr(chrono::vehicle::RotationalDamperSuspension)

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

#ifdef SWIGCSHARP
%import "chrono_swig/interface/core/ChShaft.i"
#endif

#ifdef SWIGPYCHRONO
%import(module = "pychrono.core") "chrono_swig/interface/core/ChShaft.i"
#endif

%import "../../../chrono_vehicle/ChPart.h"

// Parse the header file to generate wrappers
%include "../../../chrono_vehicle/tracked_vehicle/ChSprocket.h"
%include "../../../chrono_vehicle/tracked_vehicle/sprocket/ChSprocketSinglePin.h"
%include "../../../chrono_vehicle/tracked_vehicle/sprocket/ChSprocketDoublePin.h"
%include "../../../chrono_vehicle/tracked_vehicle/sprocket/ChSprocketBand.h"
%include "../../../chrono_vehicle/tracked_vehicle/sprocket/SprocketSinglePin.h"
%include "../../../chrono_vehicle/tracked_vehicle/sprocket/SprocketDoublePin.h"
%include "../../../chrono_vehicle/tracked_vehicle/sprocket/SprocketBand.h"

%ignore chrono::vehicle::ChIdler::GetLocation;
%include "../../../chrono_vehicle/tracked_vehicle/ChIdler.h"
%include "../../../chrono_vehicle/tracked_vehicle/idler/ChTranslationalIdler.h"
//%include "../../../chrono_vehicle/tracked_vehicle/idler/ChDistanceIdler.h"
%include "../../../chrono_vehicle/tracked_vehicle/idler/TranslationalIdler.h"
//%include "../../../chrono_vehicle/tracked_vehicle/idler/DistanceIdler.h"

%include "../../../chrono_vehicle/tracked_vehicle/ChTrackWheel.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_wheel/ChSingleTrackWheel.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_wheel/ChDoubleTrackWheel.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_wheel/SingleTrackWheel.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_wheel/DoubleTrackWheel.h"

%ignore chrono::vehicle::ChTranslationalDamperSuspension::GetLocation;
%include "../../../chrono_vehicle/tracked_vehicle/ChTrackSuspension.h"
%include "../../../chrono_vehicle/tracked_vehicle/suspension/ChTranslationalDamperSuspension.h"
%include "../../../chrono_vehicle/tracked_vehicle/suspension/ChRotationalDamperSuspension.h"
%include "../../../chrono_vehicle/tracked_vehicle/suspension/TranslationalDamperSuspension.h"
%include "../../../chrono_vehicle/tracked_vehicle/suspension/RotationalDamperSuspension.h"

%include "../../../chrono_vehicle/tracked_vehicle/ChTrackShoe.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBand.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandANCF.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandBushing.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSegmented.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSinglePin.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeBandANCF.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeBandBushing.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeSinglePin.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeDoublePin.h"

%include "../../../chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBand.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBandANCF.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBandBushing.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySegmented.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySinglePin.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyDoublePin.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyBandANCF.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyBandBushing.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblySinglePin.h"
%include "../../../chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyDoublePin.h"

%include "../../../chrono_vehicle/tracked_vehicle/ChTrackBrake.h"
%include "../../../chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeSimple.h"
%include "../../../chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeShafts.h"
%include "../../../chrono_vehicle/tracked_vehicle/brake/TrackBrakeSimple.h"
%include "../../../chrono_vehicle/tracked_vehicle/brake/TrackBrakeShafts.h"

%include "../../../chrono_vehicle/tracked_vehicle/ChTrackContactManager.h"

%include "../../../chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"
%include "../../../chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"

%include "chrono_swig/interface/models/TrackAssemblyModels.i"
