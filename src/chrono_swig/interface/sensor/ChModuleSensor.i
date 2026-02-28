//////////////////////////////////////////////////
//
//   ChModuleSensor.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the C::E
//   wrapper for Python.
//
///////////////////////////////////////////////////




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
//%include "std_unique_ptr.i"


// Include C++ headers this way...

%{

#define SWIG_FILE_WITH_INIT

#include <memory>

#include "chrono_sensor/ChConfigSensor.h"

#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualShapeBarrel.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCone.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeRoundedCylinder.h"
#include "chrono/assets/ChVisualShapeRoundedBox.h"
#include "chrono/assets/ChVisualShapePath.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/assets/ChVisualShapePointPoint.h"
#include "chrono/assets/ChVisualShapeSurface.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/sensors/Sensor.h"

#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChSensorBuffer.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"

#ifdef CHRONO_HAS_OPTIX

  #include "chrono_sensor/sensors/ChOptixSensor.h"
  #include "chrono_sensor/sensors/ChCameraSensor.h"
  #include "chrono_sensor/sensors/ChSegmentationCamera.h"
  #include "chrono_sensor/sensors/ChDepthCamera.h"
  #include "chrono_sensor/sensors/ChLidarSensor.h"
  #include "chrono_sensor/sensors/ChRadarSensor.h"
  #include "chrono_sensor/optix/scene/ChScene.h"
  #include "chrono_sensor/optix/ChOptixDefinitions.h"
  #include "chrono_sensor/optix/ChOptixUtils.h"

#endif

using namespace chrono;
using namespace chrono::sensor;

%}


// Undefine ChApi and other macros that otherwise SWIG gives a syntax error
#define CH_SENSOR_API
#define ChApi
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW

%ignore CH_ENUM_MAPPER_BEGIN;
%ignore CH_ENUM_VAL;
%ignore CH_ENUM_MAPPER_END;
%ignore CH_CLASS_VERSION;

// Include other .i configuration files for SWIG.
// These are divided in many .i files, each per a
// different c++ class, when possible.

%include "std_string.i"
%include "std_wstring.i"
%include "std_vector.i"
%include "std_list.i"
%include "carrays.i"
%include "typemaps.i"
%include "wchar.i"
#ifdef SWIGPYTHON   // --------------------------------------------------------------------- PYTHON
%include "python/cwstring.i"
%include "cstring.i"
#ifdef CHRONO_PYTHON_NUMPY
%include "../numpy.i"
#endif
#endif              // --------------------------------------------------------------------- PYTHON
%include "stdint.i"
%include "cpointer.i"

#ifdef SWIGPYTHON
#ifdef CHRONO_PYTHON_NUMPY
%init %{
    import_array();
%}

%apply (double** ARGOUTVIEW_ARRAY1, int *DIM1) {(double** vec, int* n)};
%apply ( float** ARGOUTVIEW_ARRAY3, int* DIM1, int* DIM2, int* DIM3) {(float** vec, int* h, int* w, int* c)};
%apply ( uint8_t** ARGOUTVIEW_ARRAY3, int* DIM1, int* DIM2, int* DIM3) {(uint8_t** vec, int* h, int* w, int* c)};

#endif
#endif

// This is to enable references to double,int,etc. types in function parameters
%pointer_class(int,int_ptr);
%pointer_class(double,double_ptr);
%pointer_class(float,float_ptr);
%pointer_class(char,char_ptr);


//
// For each class, keep updated the  A, B, C sections:
//

//%template(ChFramedList) std::vector<chrono::ChFrame<double>> ;


//
// A- ENABLE SHARED POINTERS
//
// Note that this must be done for almost all objects (not only those that are
// handled by shered pointers in C++, but all their chidren and parent classes. It
// is enough that a single class in an inheritance tree uses %shared_ptr, and all other in the
// tree must be promoted to %shared_ptr too).



%shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<char[]>>)
%shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::AccelData[]>>)
%shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GyroData[]>>)
%shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::MagnetData[]>>)
%shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>>)

#ifdef CHRONO_HAS_OPTIX

  %shared_ptr(chrono::sensor::ChScene)
  %shared_ptr(chrono::sensor::PixelDI)
  %shared_ptr(chrono::sensor::PixelXYZI)
  %shared_ptr(chrono::sensor::PixelRGBA8)
  %shared_ptr(chrono::sensor::PixelDepth)
  %shared_ptr(chrono::sensor::PixelSemantic)

  %shared_ptr(chrono::sensor::LidarBufferT)
  %shared_ptr(chrono::sensor::RadarBufferT)

  %shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>>)
  %shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>>)
  %shared_ptr(chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>>)
  %shared_ptr(chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>>)
  %shared_ptr(chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarReturn[]>>)
  %shared_ptr(chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarXYZReturn[]>>)
  %shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>>)
  %shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelHalf4[]>>)
  %shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDepth[]>>)
  %shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelSemantic[]>>)

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
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChFrame.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChBody.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChVector2.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChVector3.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChColor.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChColormap.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChVisualBSDFType.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChVisualMaterial.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChVisualShape.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChVisualModel.i"

%template(vector_ChFramed) std::vector< chrono::ChFrame<double> >;
%template(vector_double) std::vector<double> ;

%include "ChSensorBuffer.i"
%include "ChFilter.i"
%include "ChSensor.i"
%include "ChGPSSensor.i"
%include "ChIMUSensor.i"
#ifdef CHRONO_HAS_OPTIX
  %include "ChOptixSensor.i"
#endif
%include "chrono_sensor/ChSensorManager.h"
%include "chrono_sensor/sensors/ChNoiseModel.h"




// ADD PYTHON CODE

/*
%pythoncode %{

%}
*/
