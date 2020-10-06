//////////////////////////////////////////////////
//
//   ChModuleSensor.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the C::E
//   wrapper for Python.
//
///////////////////////////////////////////////////



// Define the module to be used in Python when typing
//  'import pychrono.sensor'


%module(directors="1") sensor


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
//%include "std_unique_ptr.i"


// Include C++ headers this way...

%{
#define SWIG_FILE_WITH_INIT
#include <memory>
#include "chrono/solver/ChSolver.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/solver/ChSolver.h"
#include "chrono/solver/ChIterativeSolver.h"


//#include "chrono/assets/ChTriangleMeshShape.h"
//#include "chrono/core/ChFrame.h"


//#include <irrlicht.h>
#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/ChOptixSensor.h"
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChConfigSensor.h.in"
#include "chrono_sensor/ChGPSSensor.h"
#include "chrono_sensor/ChIMUSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/ChSensorBuffer.h"
#include "chrono_sensor/scene/ChScene.h"
/// FILTERS
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGPSUpdate.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterIMUUpdate.h"
#include "chrono_sensor/filters/ChFilterOptixRender.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChIMUSensor.h"
#include "chrono_sensor/ChGPSSensor.h"
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilter.h"


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
%include "std_vector.i"
%include "std_list.i"
%include "carrays.i"
%include "typemaps.i"
%include "wchar.i"
%include "python/cwstring.i"
%include "cstring.i"
%include "stdint.i"
%include "numpy.i"

%init %{
    import_array();
%}
// This is to enable references to double,int,etc. types in function parameters
%pointer_class(int,int_ptr);
%pointer_class(double,double_ptr);
%pointer_class(float,float_ptr);
%pointer_class(char,char_ptr);

%apply (double** ARGOUTVIEW_ARRAY1, int *DIM1) {(double** vec, int* n)};
%apply ( float** ARGOUTVIEW_ARRAY3, int* DIM1, int* DIM2, int* DIM3) {(float** vec, int* h, int* w, int* c)};
%apply ( uint8_t** ARGOUTVIEW_ARRAY3, int* DIM1, int* DIM2, int* DIM3) {(uint8_t** vec, int* h, int* w, int* c)};

//
// For each class, keep updated the  A, B, C sections:
//

//%template(ChFrameDList) std::vector<chrono::ChFrame<double>> ;
//%template(ChFrameDList) std::vector< std::shared_ptr<chrono::ChAsset> >;


//
// A- ENABLE SHARED POINTERS
//
// Note that this must be done for almost all objects (not only those that are
// handled by shered pointers in C++, but all their chidren and parent classes. It
// is enough that a single class in an inheritance tree uses %shared_ptr, and all other in the
// tree must be promoted to %shared_ptr too).

%shared_ptr(chrono::ChFrame<double>)

%shared_ptr(chrono::sensor::ChSensor)
%shared_ptr(chrono::sensor::ChScene)
%shared_ptr(chrono::sensor::ChOptixSensor)
%shared_ptr(chrono::sensor::ChLidarSensor)
%shared_ptr(chrono::sensor::ChIMUSensor)
%shared_ptr(chrono::sensor::ChGPSSensor)
%shared_ptr(chrono::sensor::ChCameraSensor)

%shared_ptr(chrono::ChTriangleMeshShape)

%shared_ptr(chrono::sensor::IMUData)
%shared_ptr(chrono::sensor::PixelDI)
%shared_ptr(chrono::sensor::PixelXYZI)

%shared_ptr(chrono::sensor::PixelRGBA8)
%shared_ptr(chrono::sensor::GPSData)

%shared_ptr(chrono::sensor::SensorBuffer)
/// Since BufferT inherits from SensorBuffer define as shared_ptrs all BufferT instances
%shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::IMUData[]>>)
%shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>>)
%shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>>)

%shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<char[]>>)
%shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>>)
%shared_ptr(chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>>)

%shared_ptr(chrono::sensor::ChFilter)
%shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::IMUData [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::IMUData [] > > > > )
%shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::PixelXYZI [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::PixelXYZI [] > > > > )
%shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::PixelDI [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::PixelDI [] > > > > )

%shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< char [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< char [] > > > > )
%shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::PixelRGBA8 [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::PixelRGBA8 [] > > > > )
%shared_ptr(chrono::sensor::ChFilterAccess< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::GPSData [] > >,std::shared_ptr< chrono::sensor::SensorBufferT< std::shared_ptr< chrono::sensor::GPSData [] > > > > )


%shared_ptr(chrono::sensor::ChFilterVisualizePointCloud)
%shared_ptr(chrono::sensor::ChFilterVisualize)
%shared_ptr(chrono::sensor::ChFilterSave)
%shared_ptr(chrono::sensor::ChFilterSavePtCloud)
%shared_ptr(chrono::sensor::ChFilterOptixRender)
%shared_ptr(chrono::sensor::ChFilterPCfromDepth)
%shared_ptr(chrono::sensor::ChFilterIMUUpdate)
%shared_ptr(chrono::sensor::ChIMUNoiseModel)
%shared_ptr(chrono::sensor::ChIMUNoiseNone)
%shared_ptr(chrono::sensor::ChIMUNoiseNormalDrift)

%shared_ptr(chrono::sensor::ChFilterGrayscale)
%shared_ptr(chrono::sensor::ChFilterGPSUpdate)
%shared_ptr(chrono::sensor::ChGPSNoiseModel)
%shared_ptr(chrono::sensor::ChGPSNoiseNone)
%shared_ptr(chrono::sensor::ChGPSNoiseNormal)

%shared_ptr(chrono::sensor::ChFilterImgAlias)
%shared_ptr(chrono::sensor::ChFilterImageResize)
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

%import(module = "pychrono.core")  "../core/ChClassFactory.i"
%import(module = "pychrono.core")  "../core/ChSystem.i"
%import(module = "pychrono.core")  "../core/ChFrame.i"
%import(module = "pychrono.core")  "../core/ChBody.i"
%import(module = "pychrono.core")  "../core/ChVector.i"
%import(module = "pychrono.core") "chrono/assets/ChTriangleMeshShape.h"

%template(vector_ChFrameD) std::vector< chrono::ChFrame<double> >;


%include "chrono_sensor/ChSensorBuffer.h"
/// BufferT Templates

%template(SensorHostIMUBuffer) chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::IMUData[]>>;
//%template(UserIMUBufferPtr) std::shared_ptr<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::IMUData[]>>>;
%template(SensorHostXYZIBuffer) chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>>;
%template(PixelDIBuffer) std::shared_ptr<chrono::sensor::PixelXYZI[]>;
%template(SensorHostDIBuffer) chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>>;

%template(DeviceRGBA8BufferPtr) chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>>;
%template(SensorDeviceRGBA8Buffer) std::shared_ptr<chrono::sensor::PixelRGBA8[]>;

%template(SensorDeviceR8Buffer) chrono::sensor::SensorBufferT<std::shared_ptr<char[]>>;
%template(DeviceR8BufferPtr) std::shared_ptr<char[]>;

%template(SensorHostGPSBuffer) chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>>;

//%template(m_filters) std::list<std::shared_ptr<chrono::sensor::ChFilter> > ;
%template(doublevec) std::vector<double> ;

/// FILTERS
//%feature("director") chrono::sensor::ChFilter;
%include "chrono_sensor/filters/ChFilter.h"
%include "chrono_sensor/filters/ChFilterAccess.h"
%include "chrono_sensor/filters/ChFilterGPSUpdate.h"
%include "chrono_sensor/filters/ChFilterGrayscale.h"
%include "chrono_sensor/filters/ChFilterIMUUpdate.h"
%include "chrono_sensor/filters/ChFilterOptixRender.h"
%include "chrono_sensor/filters/ChFilterPCfromDepth.h"
%include "chrono_sensor/filters/ChFilterSave.h"
%include "chrono_sensor/filters/ChFilterSavePtCloud.h"
%include "chrono_sensor/filters/ChFilterVisualize.h"
%include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
%include "chrono_sensor/filters/ChFilterImageOps.h"

%include "chrono_sensor/scene/ChScene.h"

%include "chrono_sensor/ChSensor.h"
%include "chrono_sensor/ChOptixSensor.h"
%include "chrono_sensor/ChCameraSensor.h"
%include "chrono_sensor/ChConfigSensor.h.in"
%include "chrono_sensor/ChGPSSensor.h"
%include "chrono_sensor/ChIMUSensor.h"
%include "chrono_sensor/ChLidarSensor.h"
%include "chrono_sensor/ChSensorManager.h"


/// Filter acces templates instances
/// Using shared ptr so far
%template(ChFilterIMUAccess) chrono::sensor::ChFilterAccess<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::IMUData[]>>, std::shared_ptr<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::IMUData[]>>>>;
%template(ChFilterDIAccess) chrono::sensor::ChFilterAccess<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>>, std::shared_ptr<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>>>>;
%template(ChFilterXYZIAccess) chrono::sensor::ChFilterAccess<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>>, std::shared_ptr<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>>>>;


%template(ChFilterRGBA8Access) chrono::sensor::ChFilterAccess<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>>, std::shared_ptr<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>>>>;

%template(ChFilterR8Access) chrono::sensor::ChFilterAccess<chrono::sensor::SensorBufferT<std::shared_ptr<char[]>>, std::shared_ptr<chrono::sensor::SensorBufferT<std::shared_ptr<char[]>>>>;

%template(ChFilterGPSAccess) chrono::sensor::ChFilterAccess<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>>, std::shared_ptr<chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>>>>;


%template(GetMostRecentDIBuffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>>> > ;
%template(GetMostRecentRGBA8Buffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>>> > ;
%template(GetMostRecentR8Buffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::SensorBufferT<std::shared_ptr<char[]>>> > ;
%template(GetMostRecentXYZIBuffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>>> > ;
%template(GetMostRecentIMUBuffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::IMUData[]>>> > ;
%template(GetMostRecentGPSBuffer) chrono::sensor::ChSensor::GetMostRecentBuffer< std::shared_ptr < chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>>> > ;


//
// ADDITIONAL C++ FUNCTIONS / CLASSES THAT ARE USED ONLY FOR PYTHON WRAPPER
//
////
////    PixelDI Extension
////
%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>> {
        public:
        bool HasData() {
            return !($self->Buffer==NULL);
        }
};


%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>> {
public:
void GetDIData(float** vec, int* h, int* w, int* c) {
    *h = $self->Height;
    *w = $self->Width;
    *c = sizeof(PixelDI)/sizeof(float);
    *vec = reinterpret_cast<float*>($self->Buffer.get());
}
};
////
////    PixelRGBA8 Extension
////
%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>> {
public:
bool HasData() {
    return !($self->Buffer==NULL);
}
};


%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>> {
public:
void GetRGBA8Data(uint8_t** vec, int* h, int* w, int* c) {
    *h = $self->Height;
    *w = $self->Width;
    *c = sizeof(PixelRGBA8)/sizeof(unsigned char);
    *vec = reinterpret_cast<uint8_t*>($self->Buffer.get());
}
};
////
////    PixelXYZI Extension
////
%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>> {
public:
bool HasData() {
    return !($self->Buffer==NULL);
}
};


%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>> {
public:
void GetXYZIData(float** vec, int* h, int* w, int* c) {
    *h = $self->Height;
    *w = $self->Width;
    *c = sizeof(PixelXYZI)/sizeof(float);
    *vec = reinterpret_cast<float*>($self->Buffer.get());
}
};
////
////    IMUData Extension
////
%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::IMUData[]>> {
public:
bool HasData() {
    return !($self->Buffer==NULL);
}
};


%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::IMUData[]>> {
public:
void GetIMUData(double** vec, int* n) {
    *n = sizeof(IMUData)/sizeof(double);
    *vec = reinterpret_cast<double*>($self->Buffer.get());
}
};
////
////    GPSData Extension
////
%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>> {
public:
bool HasData() {
    return !($self->Buffer==NULL);
}
};


%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>> {
public:
void GetGPSData(double** vec, int* n) {
    *n = sizeof(GPSData)/sizeof(double);
    *vec = reinterpret_cast<double*>($self->Buffer.get());
}
};
////
////    char8 Extension
////
%extend chrono::sensor::SensorBufferT<std::shared_ptr<char[]>> {
        public:
        bool HasData() {
            return !($self->Buffer==NULL);
        }
};


%extend chrono::sensor::SensorBufferT<std::shared_ptr<char[]>> {
        public:
        void GetChar8Data(uint8_t** vec, int* h, int* w, int* c) {
            *h = $self->Height;
            *w = $self->Width;
            *c = 1;
            *vec = reinterpret_cast<uint8_t*>($self->Buffer.get());
        }
};
//
// ADD PYTHON CODE
//

/*
%pythoncode %{

%}
*/
