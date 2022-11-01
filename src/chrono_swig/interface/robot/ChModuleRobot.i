//////////////////////////////////////////////////
//  
//   ChModuleVehicle.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the C::E
//   wrapper for Python.
//
///////////////////////////////////////////////////



// Define the module to be used in Python when typing 
//  'import pychrono.robot'


%module(directors="1") robot


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



// Include C++ headers this way...

%{
#include <string>
#include <vector>

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/solver/ChSolver.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChPhysicsItem.h"

#include "Eigen/src/Core/util/Memory.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/robot/robosimian/RoboSimian.h"
#include "chrono_models/robot/viper/Viper.h"
#include "chrono_models/robot/curiosity/Curiosity.h"
#include "chrono_models/robot/turtlebot/Turtlebot.h"
#include "chrono_models/robot/copters/Copter.h"
#include "chrono_models/robot/copters/Little_Hexy.h"

using namespace chrono;
using namespace chrono::robosimian;
using namespace chrono::viper;
using namespace chrono::curiosity;
using namespace chrono::turtlebot;
using namespace chrono::copter;

%}


// Undefine ChApiFea otherwise SWIG gives a syntax error
#define ChApi
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#define CH_MODELS_API




// Include other .i configuration files for SWIG. 
// These are divided in many .i files, each per a
// different c++ class, when possible.

%include "std_string.i"
%include "std_vector.i"
%include "std_array.i"
%include "typemaps.i"
%include "cpointer.i"

// This is to enable references to double,int,etc. types in function parameters
%pointer_class(int,int_ptr);
%pointer_class(double,double_ptr);
%pointer_class(float,float_ptr);


%template(vector_int) std::vector< int >;
%template(limb_data) std::array<double, 8>;
%template(Actuation) std::array<std::array<double, 8>, 4>;

//
// For each class, keep updated the  A, B, C sections: 
// 


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

%shared_ptr(chrono::robosimian::RS_Part)
%shared_ptr(chrono::robosimian::RS_Chassis)
%shared_ptr(chrono::robosimian::RS_Sled)
%shared_ptr(chrono::robosimian::RS_WheelDD)
%shared_ptr(chrono::robosimian::RS_Driver)

%shared_ptr(chrono::viper::ViperPart)
%shared_ptr(chrono::viper::ViperChassis)
%shared_ptr(chrono::viper::ViperWheel)
%shared_ptr(chrono::viper::ViperUpperArm)
%shared_ptr(chrono::viper::ViperLowerArm)
%shared_ptr(chrono::viper::ViperUpright)
%shared_ptr(chrono::viper::ViperDriver)
%shared_ptr(chrono::viper::ViperDCMotorControl)
%shared_ptr(chrono::viper::ViperSpeedDriver)

%shared_ptr(chrono::curiosity::CuriosityPart)
%shared_ptr(chrono::curiosity::CuriosityChassis)
%shared_ptr(chrono::curiosity::CuriosityWheel)
%shared_ptr(chrono::curiosity::CuriosityRocker)
%shared_ptr(chrono::curiosity::CuriosityBogie)
%shared_ptr(chrono::curiosity::CuriosityUpright)
%shared_ptr(chrono::curiosity::CuriosityDifferentialBar)
%shared_ptr(chrono::curiosity::CuriosityDifferentialLink)
%shared_ptr(chrono::curiosity::CuriosityDriver)
%shared_ptr(chrono::curiosity::CuriosityDCMotorControl)
%shared_ptr(chrono::curiosity::CuriositySpeedDriver)

%shared_ptr(chrono::turtlebot::Turtlebot_Part)
%shared_ptr(chrono::turtlebot::Turtlebot_Chassis)
%shared_ptr(chrono::turtlebot::Turtlebot_ActiveWheel)
%shared_ptr(chrono::turtlebot::Turtlebot_PassiveWheel)
%shared_ptr(chrono::turtlebot::Turtlebot_Rod_Short)
%shared_ptr(chrono::turtlebot::Turtlebot_BottomPlate)
%shared_ptr(chrono::turtlebot::Turtlebot_MiddlePlate)
%shared_ptr(chrono::turtlebot::Turtlebot_TopPlate)
%shared_ptr(chrono::turtlebot::Turtlebot_Rod_Long)

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
//%import(module = "pychrono.core")  "chrono_swig/interface/core/ChSystemNSC.i"
//%import(module = "pychrono.core")  "chrono_swig/interface/core/ChSystemSMC.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChAssembly.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChCoordsys.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChMatrix.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChBodyFrame.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChBody.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChBodyAuxRef.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChLinkBase.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChLinkLock.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChLinkTSDA.i"
%import(module = "pychrono.core") "../chrono/motion_functions/ChFunction_Base.h"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChMaterialSurface.i"
%import(module = "pychrono.core") "../chrono/fea/ChContinuumMaterial.h"
%import(module = "pychrono.core") "../chrono/physics/ChPhysicsItem.h"

%import(module = "pychrono.core") "../chrono/physics/ChBodyFrame.h"
%import(module = "pychrono.core") "../chrono/physics/ChLinkBase.h"
%import(module = "pychrono.core") "../chrono/assets/ChTriangleMeshShape.h"

%rename(CollisionFamily_CHASSIS) chrono::robosimian::CollisionFamily::CHASSIS;
%rename(CollisionFamily_SLED) chrono::robosimian::CollisionFamily::SLED;
%rename(VisualizationType_NONE) chrono::robosimian::VisualizationType::NONE;
%rename(VisualizationType_MESH) chrono::robosimian::VisualizationType::MESH;
%rename(VisualizationType_COLLISION) chrono::robosimian::VisualizationType::COLLISION;
%rename(CollisionFlags_COLLISION) chrono::robosimian::CollisionFlags::CHASSIS;

%ignore chrono::robosimian::RS_Driver::GetCurrentPhase;
%feature("director")  chrono::robosimian::RS_Driver::PhaseChangeCallback;

%include "../../../chrono_models/robot/robosimian/RoboSimian.h"
%include "../../../chrono_models/robot/viper/Viper.h"
%include "../../../chrono_models/robot/curiosity/Curiosity.h"
%include "../../../chrono_models/robot/turtlebot/Turtlebot.h"

%include "../../../chrono_models/robot/copters/Copter.h"
%template(ChCopter6) chrono::copter::Copter<6>;
%template(ChCopter4) chrono::copter::Copter<4>;
%include "../../../chrono_models/robot/copters/Little_Hexy.h"



//
// C- DOWNCASTING OF SHARED POINTERS
// 
// This is not automatic in Python + SWIG, except if one uses the 
// %downcast_output_sharedptr(...) macro, as above, but this causes
// a lot of code bloat. 
// Alternatively, in the following we create a set of Python-side
// functions to perform casting by hand, thank to the macro 
// %DefSharedPtrDynamicDowncast(base,derived). 
// Do not specify the "chrono::" namespace before base or derived!
// Later, in python, you can do the following:
//  myvis = chrono.CastToChVisualizationShared(myasset)
//  print ('Could be cast to visualization object?', !myvis.IsNull())

//%DefSharedPtrDynamicDowncast2NS(chrono,chrono::fea,ChPhysicsItem,ChMesh)


//
// ADDITIONAL C++ FUNCTIONS / CLASSES THAT ARE USED ONLY FOR PYTHON WRAPPER
//

/*
%inline %{


%}
*/

%extend chrono::viper::Viper{
		public:
			Viper(chrono::ChSystem* system){
			   
			   auto selfpoint = std::make_shared<chrono::viper::Viper>(system, ViperWheelType::RealWheel);
			   return selfpoint.get();
			   }
		};

%extend chrono::curiosity::Curiosity{
		public:
			Curiosity(chrono::ChSystem* system){
			   
			   auto selfpoint = std::make_shared<chrono::curiosity::Curiosity>(system, CuriosityChassisType::FullRover, CuriosityWheelType::RealWheel);
			   return selfpoint.get();
			   }
		};

%extend chrono::turtlebot::TurtleBot{
		public:
			TurtleBot(chrono::ChSystem* system,
               const chrono::ChVector<double>& robot_pos,
               const chrono::ChQuaternion<double>& robot_rot){
			   
			   auto selfpoint = std::make_shared<chrono::turtlebot::TurtleBot>(system, robot_pos, robot_rot, nullptr);
			   return selfpoint.get();
			   }
		};
//
// ADD PYTHON CODE
//

/*
%pythoncode %{

%}
*/


