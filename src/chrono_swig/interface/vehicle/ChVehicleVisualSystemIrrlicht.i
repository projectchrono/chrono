#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

%csmethodmodifiers chrono::vehicle::ChVehicleVisualSystemIrrlicht::Initialize "public override"
%csmethodmodifiers chrono::vehicle::ChVehicleVisualSystemIrrlicht::Render "public override"
%csmethodmodifiers chrono::vehicle::ChVehicleVisualSystemIrrlicht::Advance "public override"

#endif             // --------------------------------------------------------------------- CSHARP

%{
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "chrono_vehicle/visualization/ChVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemIRR.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace irr::scene; // This is inserted for the extend functions that use it

// InteractiveDriverIRR includes
#ifdef SWIGCSHARP
    #include "chrono_vehicle/driver/ChInteractiveDriver.h"
#endif

%}

// Expose for both python and csharp
%shared_ptr(chrono::vehicle::ChSuspensionTestRig)
%shared_ptr(chrono::vehicle::ChSuspensionTestRigPlatform)
%shared_ptr(chrono::vehicle::ChSuspensionTestRigPushrod)

#ifdef SWIGCSHARP

    // InteractiveDriverIRR
    %include "../../../chrono/core/ChBezierCurve.h"
    %import "ChDriver.i" // make SWIG aware of the ChDriver interface file
    %shared_ptr(chrono::vehicle::ChInteractiveDriver)

    %ignore chrono::vehicle::ChJoystickAxisIRR; // Ignore this for now Using an alias enum, SWIG can't translate the irr namespace right.

    //
    // The Vehicle Visual System
    #define CH_VEHICLE_API
    #define ChApiIrr // Placing here to remove SWIG error when calling
    // Inform SWIG about existing sharedpointers and associated inheritances from the irrlicht csharp module
    // Otherwise it struggles with the abstract class
    %import "../../../chrono_swig/interface/irrlicht/ChVisualSystemIrrlicht.i"

    // Set up shared pointers prior to header inclusions
    %shared_ptr(chrono::irrlicht::ChVisualSystemIrrlicht)
    %shared_ptr(chrono::vehicle::ChVehicleVisualSystemIrrlicht)
    %shared_ptr(chrono::vehicle::ChTrackedVehicleVisualSystemIrrlicht)
    %shared_ptr(chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht)
    %shared_ptr(chrono::vehicle::ChSuspensionTestRigVisualSystemIRR)

    // Process headers - SWIG gets very confused between the various namespaces. Tricky to solve easily.
    // As it stands, with this ordering, an instance of ChWheeledVehicleVisualSystemIrrlicht currently has inherited access to
    // methods from - ChVisualSystem, ChVehicleVisualSystem, ChVehicleVisualSystemIrrlicht, ChWheeledVehicleVisualSystemIrrlicht
    //
    // BUT not to the non-virtual methods in ChVisualSystemIrrlicht.h
    // i.e. AddSkyBox
    // Other methods from ChVisualSystemIrrlicht are tagged by intellisense as inherited from ChVisualSystem
    //
    // Extending the missing functions is a workaround
    //

    // Visual Systems, in SWIG order, 1- ChVehicleVisualSystem, 2- ChVehicleVisualSystemIrrlicht, 3 - ChVisualSystemIrrlicht
    %include "../../../chrono_vehicle/ChVehicleVisualSystem.h"  
    %include "../../../chrono_vehicle/visualization/ChVehicleVisualSystemIrrlicht.h"
    %include "../../../chrono_irrlicht/ChVisualSystemIrrlicht.h"    
    // Includes for interactive driver
    %include "../../../chrono_vehicle/driver/ChInteractiveDriver.h"

    %include "../../../chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
    %include "../../../chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
    %include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemIRR.h"

    // Manual extensions
    // extending ChVisualSystemIrrlicht methods which are in the irrlicht namespace
    // through to the vehicle namespace and attach to the WheeledVehicle class
    %extend chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht {
        void AddSkyBox(const std::string& texture_dir = GetChronoDataFile("skybox/")) {
            $self->chrono::irrlicht::ChVisualSystemIrrlicht::AddSkyBox(texture_dir);
        }
    }

    // NB: SWIG creates a swigtype for the ILightSceneNode though one already exists in the irrlicht module.
    // Ensure namespaces aren't added (i.e. irr::scene::ILightSceneNode*), or SWIG generates the same filename
    // as the existing csharp irrlicht swigtype, and the duplicate causes compile errors
    %extend chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht {
        ILightSceneNode* AddLightDirectional(double elevation = 60, double azimuth = 60, ChColor ambient = ChColor(0.5f, 0.5f, 0.5f), ChColor specular = ChColor(0.2f, 0.2f, 0.2f), ChColor diffuse = ChColor(1.0f, 1.0f, 1.0f)) {
            return $self->chrono::irrlicht::ChVisualSystemIrrlicht::AddLightDirectional(elevation, azimuth, ambient, specular, diffuse);
        }
    }

    %extend chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht {
        ILightSceneNode* AddLight(const chrono::ChVector3d& pos, double radius, chrono::ChColor color) {
            return $self->AddLight(pos, radius, color);
        }
    }

    %extend chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht {
        void SetWindowTitle(const std::string& win_title) {
            $self->chrono::irrlicht::ChVisualSystemIrrlicht::SetWindowTitle(win_title);
        }
    }

    %extend chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht {
        void AddLogo(const std::string& logo_filename = GetChronoDataFile("logo_chrono_alpha.png")) {
            $self->chrono::irrlicht::ChVisualSystemIrrlicht::AddLogo(logo_filename);
        }
    }

    %extend chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht {
        // for using irrlicht in Y-Up worlds.
        void SetCameraVertical(CameraVerticalDir vert) {
            $self->chrono::irrlicht::ChVisualSystemIrrlicht::SetCameraVertical(vert);
        }
        CameraVerticalDir GetCameraVertical() {
            return $self->chrono::irrlicht::ChVisualSystemIrrlicht::GetCameraVertical();
        }

    }
    // Visualisations for collision mesh and body frame ref
        %extend chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht {
        void EnableCollisionShapeDrawing(bool val){
            $self->chrono::irrlicht::ChVisualSystemIrrlicht::EnableCollisionShapeDrawing(val);
        }

        void EnableBodyFrameDrawing(bool val){
            $self->chrono::irrlicht::ChVisualSystemIrrlicht::EnableBodyFrameDrawing(val);
        }
    }

        // Also need to repeat for ChTrackedVehicle.

#endif

// Separate out existing Python
#ifdef SWIGPYTHON
    // Set up shared pointers prior to header inclusions
    %shared_ptr(chrono::vehicle::ChVehicleVisualSystem)
    %shared_ptr(chrono::vehicle::ChVehicleVisualSystemIrrlicht)
    %shared_ptr(chrono::vehicle::ChTrackedVehicleVisualSystemIrrlicht)
    %shared_ptr(chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht)
    %shared_ptr(chrono::vehicle::ChSuspensionTestRigVisualSystemIRR)

    %import(module = "pychrono.irrlicht") "chrono_swig/interface/irrlicht/ChVisualSystemIrrlicht.i"
    %include "../../../chrono_vehicle/ChVehicleVisualSystem.h"
    %include "../../../chrono_vehicle/visualization/ChVehicleVisualSystemIrrlicht.h"
    %include "../../../chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
    %include "../../../chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
    %include "../../../chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemIRR.h"

    //%DefSharedPtrDynamicCast2NS(chrono::irrlicht, chrono::vehicle, ChVisualSystemIrrlicht, ChVehicleVisualSystemIrrlicht)
    //%DefSharedPtrDynamicCast(chrono::vehicle, ChVehicleVisualSystem, ChVehicleVisualSystemIrrlicht)
    %DefSharedPtrDynamicCast(chrono::vehicle, ChVehicleVisualSystem, ChTrackedVehicleVisualSystemIrrlicht)
    %DefSharedPtrDynamicCast(chrono::vehicle, ChVehicleVisualSystem, ChWheeledVehicleVisualSystemIrrlicht)
    //%DefSharedPtrDynamicCast(chrono::vehicle, ChVehicleVisualSystemIrrlicht, ChTrackedVehicleVisualSystemIrrlicht)
    //%DefSharedPtrDynamicCast(chrono::vehicle, ChVehicleVisualSystemIrrlicht, ChWheeledVehicleVisualSystemIrrlicht)
    %DefSharedPtrDynamicCast2NS(chrono::irrlicht, chrono::vehicle, ChVisualSystemIrrlicht, ChSuspensionTestRigVisualSystemIRR)
#endif
