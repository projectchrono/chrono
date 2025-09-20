//////////////////////////////////////////////////
//  
//   ChModuleVsg.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the chrono::vsg3d
//   wrapper for Python.
//
///////////////////////////////////////////////////



// Define the module to be used in Python when typing 
//  'import pychrono.vsg'


%module(directors="1") vsg


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

#include "chrono/assets/ChVisualShapes.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_vsg/ChApiVSG.h"

using namespace chrono;
using namespace chrono::vsg3d;


%}

// Undefine ChApi and other macros that otherwise SWIG gives a syntax error
#define CH_VSG_API
#define ChApi 
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW

// Include other .i configuration files for SWIG. 
// These are divided in many .i files, each per a
// different c++ class, when possible.

%include "std_string.i"
%include "std_wstring.i"
%include "std_vector.i"
%include "typemaps.i"
%include "wchar.i"
#ifdef SWIGPYTHON   // --------------------------------------------------------------------- PYTHON
%include "python/cwstring.i"
#endif              // --------------------------------------------------------------------- PYTHON
%include "cstring.i"
%include "cpointer.i"

%import(module="pychrono.core") "chrono_swig/interface/core/ChClassFactory.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChVector2.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChVector3.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChMatrix.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChCoordsys.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChFrame.i"
// %import(module="pychrono.core") "../../../chrono/functions/ChFunction.h"
// %import(module="pychrono.core") "chrono_swig/interface/core/ChFunction.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChPhysicsItem.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChVisualMaterial.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChVisualShape.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChVisualModel.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChVisualSystem.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChColor.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChColormap.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChSystem.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChBodyGeometry.i"

%include "ChVisualSystemVSG.i"
