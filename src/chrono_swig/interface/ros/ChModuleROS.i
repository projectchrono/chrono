//////////////////////////////////////////////////
//  
//   ChModuleROS.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the C::E
//   wrapper for Python.
//
///////////////////////////////////////////////////



// Define the module to be used in Python when typing 
//  'import pychrono.ros'


%module(directors="1") ros


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



// Include C++ headers this way...

%{
#include "chrono/physics/ChBody.h"
#include "chrono/core/ChFrame.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/ChROSInterface.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/ChROSTFHandler.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/robot/ChROSRobotModelHandler.h"

#ifdef CHRONO_SENSOR
#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/sensors/Sensor.h"

#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"

#include "chrono_sensor/sensors/ChIMUSensor.h"

#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSIMUHandler.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"
#endif

#ifdef CHRONO_ROS_HAS_INTERFACES
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.h"
#endif

using namespace chrono;
using namespace chrono::ros;

%}

%feature("director") chrono::ros::ChROSHandler;

// Undefine ChApi and CH_ROS_API otherwise SWIG gives a syntax error
#define ChApi
#define CH_ROS_API
#define CH_SENSOR_API

#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW

// Include other .i configuration files for SWIG. 
// These are divided in many .i files, each per a
// different c++ class, when possible.

%include "std_string.i"
%include "std_vector.i"
%include "std_array.i"
%include "stdint.i"
%include "typemaps.i"
%include "cpointer.i"
%include "cdata.i"

// This is to enable references to double,int,etc. types in function parameters
%pointer_class(int,int_ptr);
%pointer_class(double,double_ptr);
%pointer_class(float,float_ptr);

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
%shared_ptr(chrono::ChBody)
%shared_ptr(chrono::ChFrame<double>)

%shared_ptr(chrono::ros::ChROSManager)
%shared_ptr(chrono::ros::ChROSHandler)
%shared_ptr(chrono::ros::ChROSInterface)
%shared_ptr(chrono::ros::ChROSClockHandler)
%shared_ptr(chrono::ros::ChROSBodyHandler)
%shared_ptr(chrono::ros::ChROSTFHandler)
%shared_ptr(chrono::ros::ChROSRobotModelHandler)

#ifdef CHRONO_SENSOR
%shared_ptr(chrono::ros::ChROSAccelerometerHandler)
%shared_ptr(chrono::ros::ChROSCameraHandler)
%shared_ptr(chrono::ros::ChROSGPSHandler)
%shared_ptr(chrono::ros::ChROSGyroscopeHandler)
%shared_ptr(chrono::ros::ChROSLidarHandler)
%shared_ptr(chrono::ros::ChROSMagnetometerHandler)
%shared_ptr(chrono::ros::ChROSIMUHandler)
#endif

#ifdef CHRONO_ROS_HAS_INTERFACES
%shared_ptr(chrono::ros::ChROSDriverInputsHandler)
%shared_ptr(chrono::ros::ChROSViperDCMotorControlHandler)
#endif


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
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChSystem.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChBody.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChFrame.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChBodyFrame.i"

#ifdef CHRONO_SENSOR
%import(module = "pychrono.sensor")  "chrono_swig/interface/sensor/ChSensor.i"
%import(module = "pychrono.sensor")  "chrono_swig/interface/sensor/ChOptixSensor.i"
%import(module = "pychrono.sensor")  "chrono_swig/interface/sensor/ChIMUSensor.i"
%import(module = "pychrono.sensor")  "chrono_swig/interface/sensor/ChGPSSensor.i"
#endif

%include "../../../chrono_ros/ChROSManager.h"

%include "../../../chrono_ros/ChROSHandler.h"
%include "../../../chrono_ros/ChROSInterface.h"
%include "../../../chrono_ros/handlers/ChROSClockHandler.h"
%include "../../../chrono_ros/handlers/ChROSBodyHandler.h"
%include "../../../chrono_ros/handlers/ChROSTFHandler.h"
%include "../../../chrono_ros/handlers/ChROSHandlerUtilities.h"
%include "../../../chrono_ros/handlers/robot/ChROSRobotModelHandler.h"

#ifdef CHRONO_SENSOR
%include "../../../chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
%include "../../../chrono_ros/handlers/sensor/ChROSCameraHandler.h"
%include "../../../chrono_ros/handlers/sensor/ChROSGPSHandler.h"
%include "../../../chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
%include "../../../chrono_ros/handlers/sensor/ChROSLidarHandler.h"
%include "../../../chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
%include "../../../chrono_ros/handlers/sensor/ChROSIMUHandler.h"
%include "../../../chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"
#endif

#ifdef CHRONO_ROS_HAS_INTERFACES
%include "../../../chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
%include "../../../chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.h"
#endif

//
// C- CASTING OF SHARED POINTERS
// 
// This is not automatic in Python + SWIG, except if one uses the 
// %downcast_output_sharedptr(...) macro, as above, but this causes
// a lot of code bloat. 
// Alternatively, in the following we create a set of Python-side
// functions to perform casting by hand, thank to the macro 
// %DefSharedPtrDynamicCast(base,derived). 
// Do not specify the "chrono::" namespace before base or derived!
// Later, in python, you can do the following:
//  myvis = chrono.CastToChVisualizationShared(myasset)
//  print ('Could be cast to visualization object?', !myvis.IsNull())

//%DefSharedPtrDynamicCast2NS(chrono,chrono::fea,ChPhysicsItem,ChMesh)


//
// ADDITIONAL C++ FUNCTIONS / CLASSES THAT ARE USED ONLY FOR PYTHON WRAPPER
//

%ignore chrono::ros::ChROSInterface::GetNode;

#ifdef SWIGPYTHON   // --------------------------------------------------------------------- PYTHON
//
// ADD PYTHON CODE
//

%pythoncode %{

class ChROSPythonManager(ChROSManager):
    def __init__(self, node_name="chrono_ros_node"):
        super().__init__(node_name)

%}
#endif              // --------------------------------------------------------------------- PYTHON

