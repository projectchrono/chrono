// =====================================================================================
//  
//   ChModuleVehicle.i
//
//   SWIG configuration file.
//   Processed with SWIG to create the Python and C# wrappers for the vehicle Chrono module.
//
// =====================================================================================

%module(directors="1") vehicle

// Turn on the documentation of members, for more intuitive IDE typing
%feature("autodoc", "1");
%feature("flatnested", "1");

// Turn on the exception handling to intercept C++ exceptions
%include "exception.i"

%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}


// For optional downcasting of polimorphic objects:
%include "../chrono_downcast.i" 

// For supporting shared pointers:
%include <std_shared_ptr.i>

%{
#include <string>
#include <vector>

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/solver/ChSolver.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsLoads.h"
#include "chrono/physics/ChShaftsFreewheel.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChLoadsXYZnode.h"
#include "chrono/physics/ChPhysicsItem.h"

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionModelBullet.h"

#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/collision/ChCollisionSystemBullet.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleOutput.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/ChWorldFrame.h"

#include "chrono_vehicle/ChPowertrain.h"

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChTerrain.h"

#include "chrono_vehicle/ChVehicleVisualSystem.h"

// Wheeled vehicle
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledTrailer.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"

#include "chrono_vehicle/wheeled_vehicle/ChAxle.h"

#include "chrono_vehicle/wheeled_vehicle/ChBrake.h"
#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"
#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeShafts.h"
#include "chrono_vehicle/wheeled_vehicle/brake/BrakeSimple.h"
#include "chrono_vehicle/wheeled_vehicle/brake/BrakeShafts.h"

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDriverSTR.h"

// Tracked vehicle
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"

#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"
#include "chrono_vehicle/tracked_vehicle/ChIdler.h"
#include "chrono_vehicle/tracked_vehicle/ChRoadWheel.h"
#include "chrono_vehicle/tracked_vehicle/ChRoadWheelAssembly.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackBrake.h"
#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeSimple.h"
#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeShafts.h"
#include "chrono_vehicle/tracked_vehicle/brake/TrackBrakeSimple.h"
#include "chrono_vehicle/tracked_vehicle/brake/TrackBrakeShafts.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackContactManager.h"

// Vehicle models
#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "Eigen/src/Core/util/Memory.h"

using namespace chrono;
using namespace chrono::vehicle;

using namespace chrono::vehicle::generic;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::vehicle::sedan;
using namespace chrono::vehicle::citybus;
using namespace chrono::vehicle::man;
using namespace chrono::vehicle::uaz;
using namespace chrono::vehicle::gator;
using namespace chrono::vehicle::man;
using namespace chrono::vehicle::fmtv;
using namespace chrono::vehicle::kraz;

using namespace chrono::vehicle::m113;
%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define CH_VEHICLE_API 
#define ChApi
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define CH_DEPRECATED(msg)
#define CH_MODELS_API

// workaround for trouble
//%ignore chrono::fea::ChContactNodeXYZ::ComputeJacobianForContactPart;

// Include other .i configuration files for SWIG. 
%include "std_string.i"
%include "std_vector.i"
%include "typemaps.i"
%include "cstring.i"

#ifdef SWIGPYTHON
%include "std_wstring.i"
%include "wchar.i"
%include "python/cwstring.i"
#endif

// This is to enable references to double,int,etc. types in function parameters
%pointer_class(int,int_ptr);
%pointer_class(double,double_ptr);
%pointer_class(float,float_ptr);
%pointer_class(char,char_ptr);

%template(vector_int) std::vector<int>;
%template(vector_double) std::vector<double>;
%template(TerrainForces) std::vector<chrono::vehicle::TerrainForce>;
%template(WheelStates) std::vector<chrono::vehicle::WheelState>;
%template(ChWheelList) std::vector<std::shared_ptr<chrono::vehicle::ChWheel> > ;
%template(ChAxleList) std::vector<std::shared_ptr<chrono::vehicle::ChAxle> > ;

//
// A- ENABLE SHARED POINTERS
//
// Note that this must be done for almost all objects (not only those that are
// handled by shered pointers in C++, but all their chidren and parent classes. It
// is enough that a single class in an inheritance tree uses %shared_ptr, and all other in the 
// tree must be promoted to %shared_ptr too).

//from core module:
%shared_ptr(chrono::ChFunction)
%shared_ptr(chrono::ChFrame<double>) 
%shared_ptr(chrono::ChFrameMoving<double>)
%shared_ptr(chrono::ChPhysicsItem)
%shared_ptr(chrono::ChNodeBase) 
%shared_ptr(chrono::ChNodeXYZ) 
%shared_ptr(chrono::ChTriangleMeshShape)
%shared_ptr(chrono::geometry::ChTriangleMeshConnected)
%shared_ptr(chrono::ChFunction_Recorder)
%shared_ptr(chrono::ChBezierCurve)
%shared_ptr(chrono::ChLinkMarkers)

%shared_ptr(chrono::collision::ChCollisionModel)
%shared_ptr(chrono::collision::ChCollisionModelBullet)
%shared_ptr(chrono::collision::ChCollisionSystem::BroadphaseCallback)
%shared_ptr(chrono::collision::ChCollisionSystem::NarrowphaseCallback)

/*
from this module: pay attention to inheritance in the model namespace (generic, sedan etc). 
If those classes are wrapped, their parents are marked as shared_ptr while they are not, SWIG can't hanlde them.
Before adding a shared_ptr, mark as shared ptr all its inheritance tree in the model namespaces
*/

%shared_ptr(chrono::vehicle::RigidTerrain::Patch)
%shared_ptr(chrono::vehicle::ChPart)
%shared_ptr(chrono::vehicle::ChWheel)
%shared_ptr(chrono::vehicle::Wheel)
%shared_ptr(chrono::vehicle::ChBrake)
%shared_ptr(chrono::vehicle::ChBrakeSimple)
%shared_ptr(chrono::vehicle::ChBrakeShafts)
%shared_ptr(chrono::vehicle::BrakeSimple)
%shared_ptr(chrono::vehicle::BrakeShafts)
%shared_ptr(chrono::vehicle::ChVehicle)
%shared_ptr(chrono::vehicle::ChAxle)
%shared_ptr(chrono::vehicle::ChWheeledVehicle)
%shared_ptr(chrono::vehicle::ChWheeledTrailer)
%shared_ptr(chrono::vehicle::WheeledVehicle)
%shared_ptr(chrono::vehicle::WheeledTrailer)

%shared_ptr(chrono::vehicle::ChVehicleVisualSystem)

%shared_ptr(chrono::vehicle::ChSuspensionTestRig)
%shared_ptr(chrono::vehicle::ChSuspensionTestRigPlatform)
%shared_ptr(chrono::vehicle::ChSuspensionTestRigPushrod)

%shared_ptr(chrono::vehicle::ChSprocket)
%shared_ptr(chrono::vehicle::ChIdler)
%shared_ptr(chrono::vehicle::ChRoadWheel)
%shared_ptr(chrono::vehicle::ChRoadWheelAssembly)
%shared_ptr(chrono::vehicle::ChTrackShoe)
%shared_ptr(chrono::vehicle::ChTrackAssembly)
%shared_ptr(chrono::vehicle::ChTrackBrake)
%shared_ptr(chrono::vehicle::ChTrackBrakeSimple)
%shared_ptr(chrono::vehicle::ChTrackBrakeShafts)
%shared_ptr(chrono::vehicle::TrackBrakeSimple)
%shared_ptr(chrono::vehicle::TrackBrakeShafts)
%shared_ptr(chrono::vehicle::ChTrackedVehicle)
%shared_ptr(chrono::vehicle::TrackedVehicle)

%shared_ptr(chrono::vehicle::ChTrackContactManager)
%shared_ptr(chrono::vehicle::ChTrackCollisionManager)
%shared_ptr(chrono::vehicle::ChTrackCustomContact)

%shared_ptr(chrono::vehicle::LinearSpringForce)
%shared_ptr(chrono::vehicle::LinearDamperForce)
%shared_ptr(chrono::vehicle::LinearSpringDamperForce)
%shared_ptr(chrono::vehicle::LinearSpringDamperActuatorForce)
%shared_ptr(chrono::vehicle::MapSpringForce)
%shared_ptr(chrono::vehicle::MapSpringBistopForce)
%shared_ptr(chrono::vehicle::LinearSpringBistopForce)
%shared_ptr(chrono::vehicle::DegressiveDamperForce)
%shared_ptr(chrono::vehicle::MapDamperForce)
%shared_ptr(chrono::vehicle::MapSpringDamperActuatorForce)
%shared_ptr(chrono::vehicle::LinearSpringTorque)
%shared_ptr(chrono::vehicle::LinearDamperTorque)
%shared_ptr(chrono::vehicle::LinearSpringDamperTorque)
%shared_ptr(chrono::vehicle::LinearSpringDamperActuatorTorque)
%shared_ptr(chrono::vehicle::MapSpringTorque)
%shared_ptr(chrono::vehicle::MapDamperTorque)

//
// B- INCLUDE HEADERS
//
//
// 1) 
//    When including with %include all the .i files, make sure that 
// the .i of a derived class is included AFTER the .i of
// a base class, otherwise SWIG is not able to build the type
// infos. 
//
// 2)
//    Then, this said, if one member function in Foo_B.i returns
// an object of Foo_A.i (or uses it as a parameter) and yet you must %include
// A before B, ex.because of rule 1), a 'forward reference' to A must be done in
// B by. Seems that it is enough to write 
//  mynamespace { class myclass; }
// in the .i file, before the %include of the .h, even if already forwarded in .h

#ifdef SWIGCSHARP
%import  "chrono_swig/interface/core/ChClassFactory.i"
%import  "chrono_swig/interface/core/ChObject.i"
%import  "chrono_swig/interface/core/ChPhysicsItem.i"
%import  "chrono_swig/interface/core/ChVector.i"
%import  "chrono_swig/interface/core/ChQuaternion.i"
%import  "chrono_swig/interface/core/ChCoordsys.i"
%import  "chrono_swig/interface/core/ChFrame.i"
%import  "chrono_swig/interface/core/ChFrameMoving.i"
%import  "chrono_swig/interface/core/ChTimestepper.i"
%import  "chrono_swig/interface/core/ChSystem.i"
%import  "chrono_swig/interface/core/ChAssembly.i"
%import  "chrono_swig/interface/core/ChCoordsys.i"
%import  "chrono_swig/interface/core/ChMatrix.i"
%import  "chrono_swig/interface/core/ChBodyFrame.i"
%import  "chrono_swig/interface/core/ChBody.i"
%import  "chrono_swig/interface/core/ChBodyAuxRef.i"
%import  "chrono_swig/interface/core/ChLinkBase.i"
%import  "chrono_swig/interface/core/ChLinkLock.i"
%import  "chrono_swig/interface/core/ChLinkTSDA.i"
%import  "chrono_swig/interface/core/ChLinkRSDA.i"
%import  "chrono_swig/interface/core/ChLoad.i"
%import  "chrono_swig/interface/core/ChShaft.i"
%import  "chrono_swig/interface/core/ChVisualShape.i"
%import  "chrono_swig/interface/core/ChContactContainer.i"
%import  "../../../chrono/motion_functions/ChFunction.h"
%import  "chrono_swig/interface/core/ChMaterialSurface.i"
%import  "../../../chrono/fea/ChContinuumMaterial.h"
%import  "../../../chrono/physics/ChPhysicsItem.h"
%import  "../../../chrono/physics/ChNodeBase.h"
%import  "../../../chrono/physics/ChBodyFrame.h"
%import  "../../../chrono/physics/ChLinkBase.h"
%import  "chrono_swig/interface/core/ChTexture.i"
%import  "../../../chrono/assets/ChTriangleMeshShape.h"
#endif

#ifdef SWIGPYTHON
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChClassFactory.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChObject.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChPhysicsItem.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChVector.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChQuaternion.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChCoordsys.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChFrame.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChFrameMoving.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChTimestepper.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChSystem.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChAssembly.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChCoordsys.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChMatrix.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChBodyFrame.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChBody.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChBodyAuxRef.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChLinkBase.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChLinkLock.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChLinkTSDA.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChLinkRSDA.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChLoad.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChShaft.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChVisualShape.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChContactContainer.i"
%import(module = "pychrono.core")  "../../../chrono/motion_functions/ChFunction.h"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChMaterialSurface.i"
%import(module = "pychrono.core")  "../../../chrono/fea/ChContinuumMaterial.h"
%import(module = "pychrono.core")  "../../../chrono/physics/ChPhysicsItem.h"
%import(module = "pychrono.core")  "../../../chrono/physics/ChNodeBase.h"
%import(module = "pychrono.core")  "../../../chrono/physics/ChBodyFrame.h"
%import(module = "pychrono.core")  "../../../chrono/physics/ChLinkBase.h"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChTexture.i"
%import(module = "pychrono.core")  "../../../chrono/assets/ChTriangleMeshShape.h"

#ifdef CHRONO_IRRLICHT

#define ChApiIrr 
#define IRRLICHT_API
#define _IRR_DEPRECATED_

%include "chrono_swig/interface/vehicle/ChVehicleVisualSystemIrrlicht.i"
#endif

#endif

// TODO: 
//%include "rapidjson.i"

//%include "../../../chrono_vehicle/ChApiVehicle.h"
%ignore chrono::vehicle::TrackedCollisionFamily::Enum;
%ignore chrono::vehicle::TrackedCollisionFamily::OutputInformation;
%ignore chrono::vehicle::TrackedCollisionFlag::Enum;
%include "../../../chrono_vehicle/ChSubsysDefs.h"
%include "../chrono_models/vehicle/ChVehicleModelDefs.h"
//TODO: conversion from std::vectors of ChVehicleOutput
%include "../../../chrono_vehicle/ChVehicleOutput.h"
%include "../../../chrono_vehicle/ChVehicleModelData.h"
%include "../../../chrono_vehicle/ChPart.h"
%include "../../../chrono_vehicle/ChWorldFrame.h"
%include "ChPowertrain.i"
%include "ChChassis.i"
%include "../../../chrono_vehicle/ChVehicle.h"
%include "ChDriver.i"
%include "ChTerrain.i"
%include "../../../chrono_vehicle/ChVehicleVisualSystem.h"

//TODO: antirollbar

// Wheeled vehicles
%include "ChSteering.i"
%include "ChSubchassis.i"
%include "ChSuspension.i"
%include "ChDriveline.i"

%include "../../../chrono_vehicle/wheeled_vehicle/ChWheel.h"
%include "../../../chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"

%include "../../../chrono_vehicle/wheeled_vehicle/ChBrake.h"
%include "../../../chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"
%include "../../../chrono_vehicle/wheeled_vehicle/brake/ChBrakeShafts.h"
%include "../../../chrono_vehicle/wheeled_vehicle/brake/BrakeSimple.h"
%include "../../../chrono_vehicle/wheeled_vehicle/brake/BrakeShafts.h"

%include "ChTire.i"

%include "../../../chrono_vehicle/wheeled_vehicle/ChAxle.h"

%include "../../../chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
%include "../../../chrono_vehicle/wheeled_vehicle/ChWheeledTrailer.h"
%include "../../../chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
%include "../../../chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"

%include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"

// Tracked vehicles
%include "ChTrackAssembly.i"

%include "../../../chrono_vehicle/tracked_vehicle/ChSprocket.h"
%include "../../../chrono_vehicle/tracked_vehicle/ChIdler.h"
%include "../../../chrono_vehicle/tracked_vehicle/ChRoadWheel.h"
%include "../../../chrono_vehicle/tracked_vehicle/ChRoadWheelAssembly.h"
%include "../../../chrono_vehicle/tracked_vehicle/ChTrackShoe.h"

%include "../../../chrono_vehicle/tracked_vehicle/ChTrackBrake.h"
%include "../../../chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeSimple.h"
%include "../../../chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeShafts.h"
%include "../../../chrono_vehicle/tracked_vehicle/brake/TrackBrakeSimple.h"
%include "../../../chrono_vehicle/tracked_vehicle/brake/TrackBrakeShafts.h"

%include "../../../chrono_vehicle/tracked_vehicle/ChTrackContactManager.h"

%include "../../../chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"
%include "../../../chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"

%include "chrono_swig/interface/models/WheelModels.i"
%include "chrono_swig/interface/models/BrakeModels.i"
%include "chrono_swig/interface/models/VehicleModels.i"

%include "vehicleUtils.i"

//
// C- DOWNCASTING OF SHARED POINTERS
//

%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChDoubleWishbone)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChMacPhersonStrut)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChLeafspringAxle)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChHendricksonPRIMAXX)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChDoubleWishboneReduced)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChMultiLink)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChRigidPinnedAxle)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChSemiTrailingArm)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChRigidSuspension)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChSolidAxle)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChThreeLinkIRS)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChToeBarLeafspringAxle)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChSolidBellcrankThreeLinkAxle)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChSolidThreeLinkAxle)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ChSingleWishbone)

//%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, MacPhersonStrut)
//%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, LeafspringAxle)
//%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, MultiLink)
//%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, SemiTrailingArm)
//%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, DoubleWishbone)
//%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, DoubleWishboneReduced)
//%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, HendricksonPRIMAXX)
//%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, SolidAxle)
//%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ThreeLinkIRS)
//%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, ToeBarLeafspringAxle)
//%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, SolidBellcrankThreeLinkAxle)
//%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, SolidThreeLinkAxle)
//%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSuspension, SingleWishbone)

%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSteering, ChPitmanArm)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSteering, ChPitmanArmShafts)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSteering, ChRackPinion)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSteering, ChRotaryArm)

%DefSharedPtrDynamicDowncast(chrono::vehicle,ChChassis, ChRigidChassis)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChChassisRear, ChRigidChassisRear)

%DefSharedPtrDynamicDowncast(chrono::vehicle,ChChassisConnector, ChChassisConnectorArticulated)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChChassisConnector, ChChassisConnectorHitch)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChChassisConnector, ChChassisConnectorTorsion)

%DefSharedPtrDynamicDowncast(chrono::vehicle,ChSubchassis, ChBalancer)

%DefSharedPtrDynamicDowncast(chrono::vehicle,ChTire, ChTMeasyTire)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChTire, ChRigidTire)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChTire, ChReissnerTire)
////%DefSharedPtrDynamicDowncast(chrono::vehicle,ChTire, ChPacejkaTire)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChTire, ChPac89Tire)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChTire, ChPac02Tire)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChTire, ChLugreTire)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChTire, ChFialaTire)

%DefSharedPtrDynamicDowncast(chrono::vehicle,ChPowertrain, SimplePowertrain)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChPowertrain, SimpleMapPowertrain)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChPowertrain, SimpleCVTPowertrain)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChPowertrain, ShaftsPowertrain)

%DefSharedPtrDynamicDowncast(chrono::vehicle,ChDriveline, ChDrivelineWV)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChDriveline, ChShaftsDriveline2WD)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChDriveline, ChShaftsDriveline4WD)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChDriveline, ChSimpleDriveline)
%DefSharedPtrDynamicDowncast(chrono::vehicle,ChDriveline, ChSimpleDrivelineXWD)
