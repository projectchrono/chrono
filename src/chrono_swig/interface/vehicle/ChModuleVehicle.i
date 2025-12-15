// =====================================================================================
//  
// ChModuleVehicle.i
// Create the Python and C# wrappers for the Chrono::Vehicle module.
//
// ATTENTION: 
// Must be included from another SWIG interface file which defines the module.
//
// =====================================================================================

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


// For optional casting of polimorphic objects:
%include "../chrono_cast.i" 

// For supporting shared pointers:
%include <std_shared_ptr.i>

%{
#include <string>
#include <vector>

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector2.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChFrame.h"
#include "chrono/solver/ChSolver.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsLoads.h"
#include "chrono/physics/ChShaftsFreewheel.h"
#include "chrono/physics/ChShaftsAppliedTorque.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChLoadsNodeXYZ.h"
#include "chrono/physics/ChPhysicsItem.h"

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/assets/ChVisualShapes.h"

#include "chrono/fea/ChMesh.h"

#include "chrono/input_output/ChOutput.h"
#include "chrono/input_output/ChCheckpoint.h"

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/ChWorldFrame.h"

#include "chrono_vehicle/ChPowertrainAssembly.h"

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChTerrain.h"
// moved up the file order to ensure this is included in this group - otherwise a build without irrlicht/vsg fails (i.e. Chrono Unity)
#include "chrono_vehicle/ChVehicleVisualSystem.h"


// Wheeled vehicle
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledTrailer.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"

#include "chrono_vehicle/wheeled_vehicle/ChAxle.h"

#include "chrono_vehicle/wheeled_vehicle/ChSpindle.h"
#include "chrono_vehicle/wheeled_vehicle/ChBrake.h"
#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"
#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeShafts.h"
#include "chrono_vehicle/wheeled_vehicle/brake/BrakeSimple.h"
#include "chrono_vehicle/wheeled_vehicle/brake/BrakeShafts.h"

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDriver.h"

// Tracked vehicle
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"

#include "chrono_vehicle/tracked_vehicle/ChIdler.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackWheel.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackSuspension.h"
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
using namespace chrono::fea;
using namespace chrono::vehicle;

using namespace chrono::vehicle::generic;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::vehicle::sedan;
using namespace chrono::vehicle::citybus;
using namespace chrono::vehicle::man;
using namespace chrono::vehicle::uaz;
using namespace chrono::vehicle::gator;
using namespace chrono::vehicle::feda;
using namespace chrono::vehicle::bmw;
using namespace chrono::vehicle::man;
using namespace chrono::vehicle::fmtv;
using namespace chrono::vehicle::kraz;

using namespace chrono::vehicle::m113;
%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define CH_VEHICLE_API 

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON
#ifdef CHRONO_FSI_SPH
#define CH_FSI_API
#endif
#endif             // --------------------------------------------------------------------- PYTHON

#define ChApi
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define CH_DEPRECATED(msg)
#define CH_MODELS_API

// workaround for trouble
//%ignore chrono::fea::ChContactNodeXYZ::ComputeJacobianForContactPart;

// Include other .i configuration files for SWIG. 
%include "std_string.i"
%include "std_vector.i"
%include "std_pair.i"
%include "typemaps.i"
#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON
%include "cstring.i"
#endif             // --------------------------------------------------------------------- PYTHON
%include "cpointer.i"

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON
%include "std_wstring.i"
%include "wchar.i"
%include "python/cwstring.i"
#endif             // --------------------------------------------------------------------- PYTHON

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
%shared_ptr(chrono::ChVisualShapeTriangleMesh)
%shared_ptr(chrono::ChTriangleMeshConnected)
%shared_ptr(chrono::ChFunctionInterp)
%shared_ptr(chrono::ChBezierCurve)
%shared_ptr(chrono::ChLinkMarkers)
%shared_ptr(chrono::ChContactable)
%shared_ptr(chrono::ChContactable_1vars)
%shared_ptr(chrono::ChContactable_2vars)
%shared_ptr(chrono::ChContactable_3vars)
%shared_ptr(chrono::fea::ChMesh)

%shared_ptr(chrono::ChCollisionModel)
%shared_ptr(chrono::ChCollisionSystem::BroadphaseCallback)
%shared_ptr(chrono::ChCollisionSystem::NarrowphaseCallback)

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON
#ifdef CHRONO_FSI_SPH
%shared_ptr(chrono::vehicle::CRMTerrain)
#endif
#endif             // --------------------------------------------------------------------- PYTHON

%import(module = "pychrono.core") "chrono_swig/interface/core/ChClassFactory.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChVector2.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChVector3.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChQuaternion.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChCoordsys.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChFrame.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChFrameMoving.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChTimestepper.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChObject.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChPhysicsItem.i"  
%import(module = "pychrono.core") "chrono_swig/interface/core/ChSystem.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChAssembly.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChMatrix.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChBodyFrame.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChBody.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChBodyAuxRef.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChNodeXYZ.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChLinkBase.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChLinkLock.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChLinkTSDA.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChLinkRSDA.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChLoad.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChShaft.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChVisualShape.i"
%import(module = "pychrono.core") "../../../chrono/geometry/ChTriangleMeshConnected.h"
%import(module = "pychrono.core") "../../../chrono/assets/ChVisualShapeTriangleMesh.h"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChContactContainer.i"
%import(module = "pychrono.core") "../../../chrono/functions/ChFunction.h"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChContactMaterial.i"
%import(module = "pychrono.core") "../../../chrono/fea/ChContinuumMaterial.h"
%import(module = "pychrono.core") "../../../chrono/physics/ChPhysicsItem.h"
%import(module = "pychrono.core") "../../../chrono/physics/ChNodeBase.h"
%import(module = "pychrono.core") "../../../chrono/physics/ChBodyFrame.h"
%import(module = "pychrono.core") "../../../chrono/physics/ChLinkBase.h"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChTexture.i"
%import(module = "pychrono.core") "../../../chrono/fea/ChMesh.h"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChBodyGeometry.i"

%import(module = "pychrono.core") "../../../chrono/input_output/ChOutput.h"
%import(module = "pychrono.core") "../../../chrono/input_output/ChCheckpoint.h"

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

#ifdef CHRONO_FSI_SPH
%import(module = "pychrono.fsi") "chrono_swig/interface/fsi/ChFsiProblemSPH.i"
#endif
#endif             // --------------------------------------------------------------------- PYTHON

#ifdef CHRONO_VSG
#define CH_VSG_API
%import(module = "pychrono.vsg3d") "chrono_swig/interface/vsg/ChVisualSystemVSG.i"
#endif


#ifdef CHRONO_IRRLICHT
#define ChApiIrr 
%import(module = "pychrono.irrlicht") "dimension2d.h"
%import(module = "pychrono.irrlicht") "../irrlicht/ChVisualSystemIrrlicht.i"
#endif

/*
from this module: pay attention to inheritance in the model namespace (generic, sedan etc). 
If those classes are wrapped, their parents are marked as shared_ptr while they are not, SWIG can't handle them.
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
%shared_ptr(chrono::vehicle::ChSpindle)
%shared_ptr(chrono::vehicle::ChWheeledVehicle)
%shared_ptr(chrono::vehicle::ChWheeledTrailer)
%shared_ptr(chrono::vehicle::WheeledVehicle)
%shared_ptr(chrono::vehicle::WheeledTrailer)

// For the powertrain base class changes
%shared_ptr(chrono::vehicle::ChEngine)
%shared_ptr(chrono::vehicle::ChEngineSimple)
%shared_ptr(chrono::vehicle::ChEngineShafts)

%shared_ptr(chrono::vehicle::ChTransmission)

%shared_ptr(chrono::vehicle::ChVehicleVisualSystem)
%shared_ptr(chrono::vehicle::ChSuspensionTestRig)
%shared_ptr(chrono::vehicle::ChSuspensionTestRigPlatform)
%shared_ptr(chrono::vehicle::ChSuspensionTestRigPushrod)

%shared_ptr(chrono::vehicle::ChDriver)
%shared_ptr(chrono::vehicle::ChSprocket)
%shared_ptr(chrono::vehicle::ChIdler)
%shared_ptr(chrono::vehicle::ChTrackWheel)
%shared_ptr(chrono::vehicle::ChTrackSuspension)
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

%shared_ptr(chrono::vehicle::SpringForce)
%shared_ptr(chrono::vehicle::LinearSpringForce)
%shared_ptr(chrono::vehicle::LinearDamperForce)
%shared_ptr(chrono::vehicle::LinearSpringDamperForce)
%shared_ptr(chrono::vehicle::NonlinearSpringForce)
%shared_ptr(chrono::vehicle::NonlinearDamperForce)
%shared_ptr(chrono::vehicle::NonlinearSpringDamperForce)
%shared_ptr(chrono::vehicle::MapSpringDamperForce)
%shared_ptr(chrono::vehicle::DegressiveDamperForce)

%shared_ptr(chrono::vehicle::LinearSpringTorque)
%shared_ptr(chrono::vehicle::LinearDamperTorque)
%shared_ptr(chrono::vehicle::LinearSpringDamperTorque)
%shared_ptr(chrono::vehicle::NonlinearSpringTorque)
%shared_ptr(chrono::vehicle::NonlinearDamperTorque)
%shared_ptr(chrono::vehicle::NonlinearSpringDamperTorque)

// Templates after all shared pointers
%template(vector_int) std::vector<int>;
%template(vector_double) std::vector<double>;
%template(TerrainForces) std::vector<chrono::vehicle::TerrainForce>;
%template(WheelStates) std::vector<chrono::vehicle::WheelState>;
%template(ChWheelList) std::vector<std::shared_ptr<chrono::vehicle::ChWheel> > ;
%template(ChAxleList) std::vector<std::shared_ptr<chrono::vehicle::ChAxle> > ;

// TODO: 
//%include "rapidjson.i"
//%include "../../../chrono_vehicle/ChApiVehicle.h"
%ignore chrono::vehicle::VehicleCollisionFamily::Enum;
%ignore chrono::vehicle::TrackedCollisionFlag::Enum;
%ignore chrono::vehicle::OutputInformation;
%ignore chrono::vehicle::VehiclePartTag;
%include "../../../chrono_vehicle/ChSubsysDefs.h"
%include "chrono_models/vehicle/ChVehicleModelDefs.h"
%include "../../../chrono_vehicle/ChVehicleDataPath.h"
%include "../../../chrono_vehicle/ChPart.h"
%include "../../../chrono_vehicle/ChWorldFrame.h"
%include "ChChassis.i"
// Changes for the powertrain modifications
%include "ChPowertrain.i"
%include "ChEngine.i"
%include "ChTransmission.i"

%include "../../../chrono_vehicle/ChVehicle.h"
%include "ChDriver.i"
%include "ChTerrain.i"
// Place these after the ChEngine and ChTransmission and ChPowertrain base wraps
%include "chrono_swig/interface/models/PowertrainModels.i"

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
%include "../../../chrono_vehicle/wheeled_vehicle/ChSpindle.h"

#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP
// Mark override methods to avoid CS0114 warnings in Unity and instruct SWIG how to generate the correct overrides of virtual
%csmethodmodifiers chrono::vehicle::ChWheeledVehicle::Synchronize(double, const DriverInputs&) "public override"
%csmethodmodifiers chrono::vehicle::ChWheeledVehicle::Synchronize(double, const DriverInputs&, const ChTerrain&) "public override"
%csmethodmodifiers chrono::vehicle::ChTrackedVehicle::Synchronize(double, const DriverInputs&) "public override"
%csmethodmodifiers chrono::vehicle::ChTrackedVehicle::Synchronize(double, const DriverInputs&, const ChTerrain&) "public override"
#endif             // --------------------------------------------------------------------- CSHARP

%include "../../../chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
%include "../../../chrono_vehicle/wheeled_vehicle/ChWheeledTrailer.h"
%include "../../../chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
%include "../../../chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"

%include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
%include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDriver.h"

// Tracked vehicles
%include "ChTrackAssembly.i"

%include "chrono_swig/interface/models/WheelModels.i"
%include "chrono_swig/interface/models/BrakeModels.i"
%include "chrono_swig/interface/models/VehicleModels.i"

%include "vehicleUtils.i"

#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP
// Import ChVisualSystem base class unconditionally (ChVehicleVisualSystem inherits from it)
// Python gets this via module imports from pychrono.irrlicht or pychrono.vsg3d into appropriate module
// but C# Unity with no visualisation module but with this vehicle module needs an unconditional
// for SWIG to understand the inheritance
%import "chrono_swig/interface/core/ChVisualSystem.i"
#endif             // --------------------------------------------------------------------- CSHARP

%include "../../../chrono_vehicle/ChVehicleVisualSystem.h" 

#ifdef CHRONO_IRRLICHT
  #define ChApiIrr 
  #define IRRLICHT_API
  #define _IRR_DEPRECATED_
  %include "ChVehicleVisualSystemIrrlicht.i"
#endif

#ifdef CHRONO_VSG
  %include "ChVehicleVisualSystemVSG.i"
#endif

//
// C- CASTING OF SHARED POINTERS
//

%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChDoubleWishbone)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChMacPhersonStrut)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChLeafspringAxle)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChHendricksonPRIMAXX)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChDoubleWishboneReduced)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChMultiLink)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChRigidPinnedAxle)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChSemiTrailingArm)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChRigidSuspension)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChSolidAxle)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChThreeLinkIRS)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChToeBarLeafspringAxle)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChSolidBellcrankThreeLinkAxle)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChSolidThreeLinkAxle)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSuspension, ChSingleWishbone)


%DefSharedPtrDynamicCast(chrono::vehicle,ChSteering, ChPitmanArm)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSteering, ChPitmanArmShafts)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSteering, ChRackPinion)
%DefSharedPtrDynamicCast(chrono::vehicle,ChSteering, ChRotaryArm)

%DefSharedPtrDynamicCast(chrono::vehicle,ChChassis, ChRigidChassis)
%DefSharedPtrDynamicCast(chrono::vehicle,ChChassisRear, ChRigidChassisRear)

%DefSharedPtrDynamicCast(chrono::vehicle,ChChassisConnector, ChChassisConnectorArticulated)
%DefSharedPtrDynamicCast(chrono::vehicle,ChChassisConnector, ChChassisConnectorHitch)
%DefSharedPtrDynamicCast(chrono::vehicle,ChChassisConnector, ChChassisConnectorTorsion)

%DefSharedPtrDynamicCast(chrono::vehicle,ChSubchassis, ChBalancer)

%DefSharedPtrDynamicCast(chrono::vehicle,ChTire, ChRigidTire)

%DefSharedPtrDynamicCast(chrono::vehicle,ChTire, ChForceElementTire)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTire, ChTMeasyTire)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTire, ChTMsimpleTire)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTire, ChPac89Tire)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTire, ChPac02Tire)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTire, ChFialaTire)

%DefSharedPtrDynamicCast(chrono::vehicle,ChTire, ChDeformableTire)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTire, ChANCFTire)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTire, ChFEATire)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTire, ChReissnerTire)

%DefSharedPtrDynamicCast(chrono::vehicle,ChEngine, ChEngineSimple)
%DefSharedPtrDynamicCast(chrono::vehicle,ChEngine, ChEngineSimpleMap)
%DefSharedPtrDynamicCast(chrono::vehicle,ChEngine, ChEngineShafts)
%DefSharedPtrDynamicCast(chrono::vehicle,ChEngine, EngineSimple)
%DefSharedPtrDynamicCast(chrono::vehicle,ChEngine, EngineSimpleMap)
%DefSharedPtrDynamicCast(chrono::vehicle,ChEngine, EngineShafts)

%DefSharedPtrDynamicCast(chrono::vehicle,ChTransmission, ChAutomaticTransmissionSimpleMap)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTransmission, ChAutomaticTransmissionShafts)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTransmission, AutomaticTransmissionSimpleMap)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTransmission, AutomaticTransmissionShafts)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTransmission, ChManualTransmissionShafts)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTransmission, ManualTransmissionShafts)

%DefSharedPtrDynamicCast(chrono::vehicle, ChTransmission, ChAutomaticTransmission);
%DefSharedPtrDynamicCast(chrono::vehicle, ChAutomaticTransmission, ChAutomaticTransmissionSimpleMap);
%DefSharedPtrDynamicCast(chrono::vehicle, ChAutomaticTransmission, ChAutomaticTransmissionShafts);
%DefSharedPtrDynamicCast(chrono::vehicle, ChTransmission, ChManualTransmission);
%DefSharedPtrDynamicCast(chrono::vehicle, ChManualTransmission, ChManualTransmissionShafts);

%DefSharedPtrDynamicCast(chrono::vehicle,ChDriveline, ChDrivelineWV)
%DefSharedPtrDynamicCast(chrono::vehicle,ChDriveline, ChShaftsDriveline2WD)
%DefSharedPtrDynamicCast(chrono::vehicle,ChDriveline, ChShaftsDriveline4WD)
%DefSharedPtrDynamicCast(chrono::vehicle,ChDriveline, ChSimpleDriveline)
%DefSharedPtrDynamicCast(chrono::vehicle,ChDriveline, ChSimpleDrivelineXWD)

%DefSharedPtrDynamicCast(chrono::vehicle,ChTerrain, FlatTerrain)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTerrain, RigidTerrain)
%DefSharedPtrDynamicCast(chrono::vehicle,ChTerrain, SCMTerrain)
