%{

/* Includes additional C++ in the wrapper code */

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

#include "chrono_models/vehicle/m113/sprocket/M113_SprocketBand.h"
#include "chrono_models/vehicle/m113/sprocket/M113_SprocketSinglePin.h"
#include "chrono_models/vehicle/m113/sprocket/M113_SprocketDoublePin.h"

#include "chrono_models/vehicle/m113/M113_Idler.h"

#include "chrono_models/vehicle/m113/M113_RoadWheel.h"

#include "chrono_models/vehicle/m113/M113_Suspension.h"

#include "chrono_models/vehicle/m113/track_shoe/M113_TrackShoeBandANCF.h"
#include "chrono_models/vehicle/m113/track_shoe/M113_TrackShoeBandBushing.h"
#include "chrono_models/vehicle/m113/track_shoe/M113_TrackShoeSinglePin.h"
#include "chrono_models/vehicle/m113/track_shoe/M113_TrackShoeDoublePin.h"

#include "chrono_models/vehicle/m113/track_assembly/M113_TrackAssemblyBandANCF.h"
#include "chrono_models/vehicle/m113/track_assembly/M113_TrackAssemblyBandBushing.h"
#include "chrono_models/vehicle/m113/track_assembly/M113_TrackAssemblySinglePin.h"
#include "chrono_models/vehicle/m113/track_assembly/M113_TrackAssemblyDoublePin.h"
%}

%shared_ptr(chrono::vehicle::m113::M113_SprocketBand)
%shared_ptr(chrono::vehicle::m113::M113_SprocketBandLeft)
%shared_ptr(chrono::vehicle::m113::M113_SprocketBandRight)
%shared_ptr(chrono::vehicle::m113::M113_SprocketSinglePin)
%shared_ptr(chrono::vehicle::m113::M113_SprocketSinglePinLeft)
%shared_ptr(chrono::vehicle::m113::M113_SprocketSinglePinRight)
%shared_ptr(chrono::vehicle::m113::M113_SprocketDoublePin)
%shared_ptr(chrono::vehicle::m113::M113_SprocketDoublePinLeft)
%shared_ptr(chrono::vehicle::m113::M113_SprocketDoublePinRight)

%shared_ptr(chrono::vehicle::m113::M113_Idler)
%shared_ptr(chrono::vehicle::m113::M113_IdlerLeft)
%shared_ptr(chrono::vehicle::m113::M113_IdlerRight)

%shared_ptr(chrono::vehicle::m113::M113_RoadWheel)
%shared_ptr(chrono::vehicle::m113::M113_RoadWheelLeft)
%shared_ptr(chrono::vehicle::m113::M113_RoadWheelRight)

%shared_ptr(chrono::vehicle::m113::M113_Suspension)

%shared_ptr(chrono::vehicle::m113::M113_TrackShoeBandANCF)
%shared_ptr(chrono::vehicle::m113::M113_TrackShoeBandBushing)
%shared_ptr(chrono::vehicle::m113::M113_TrackShoeSinglePin)
%shared_ptr(chrono::vehicle::m113::M113_TrackShoeDoublePin)

%shared_ptr(chrono::vehicle::m113::M113_TrackAssemblyBandANCF)
%shared_ptr(chrono::vehicle::m113::M113_TrackAssemblyBandBushing)
%shared_ptr(chrono::vehicle::m113::M113_TrackAssemblySinglePin)
%shared_ptr(chrono::vehicle::m113::M113_TrackAssemblyDoublePin)

// Parse the header file to generate wrappers

%import "chrono_swig/interface/vehicle/ChTrackAssembly.i"

// Model

%include "../../../chrono_models/vehicle/m113/sprocket/M113_SprocketBand.h"
%include "../../../chrono_models/vehicle/m113/sprocket/M113_SprocketSinglePin.h"
%include "../../../chrono_models/vehicle/m113/sprocket/M113_SprocketDoublePin.h"

%include "../../../chrono_models/vehicle/m113/M113_Idler.h"

%include "../../../chrono_models/vehicle/m113/M113_RoadWheel.h"

%include "../../../chrono_models/vehicle/m113/M113_Suspension.h"

%include "../../../chrono_models/vehicle/m113/track_shoe/M113_TrackShoeBandANCF.h"
%include "../../../chrono_models/vehicle/m113/track_shoe/M113_TrackShoeBandBushing.h"
%include "../../../chrono_models/vehicle/m113/track_shoe/M113_TrackShoeSinglePin.h"
%include "../../../chrono_models/vehicle/m113/track_shoe/M113_TrackShoeDoublePin.h"

%include "../../../chrono_models/vehicle/m113/track_assembly/M113_TrackAssemblyBandANCF.h"
%include "../../../chrono_models/vehicle/m113/track_assembly/M113_TrackAssemblyBandBushing.h"
%include "../../../chrono_models/vehicle/m113/track_assembly/M113_TrackAssemblySinglePin.h"
%include "../../../chrono_models/vehicle/m113/track_assembly/M113_TrackAssemblyDoublePin.h"
