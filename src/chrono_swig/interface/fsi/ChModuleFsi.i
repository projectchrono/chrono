//////////////////////////////////////////////////
//  
//   ChModuleFsi.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the chrono::fsi
//   wrapper for Python.
//
///////////////////////////////////////////////////

// Define the module to be used in Python when typing 
//  'import pychrono.fsi'
%module(directors="1") fsi


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

// Include C++ headers
%{

#include "chrono/assets/ChVisualShapes.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_fsi/ChFsiDefinitions.h"
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiSystem.h"
#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"
#include "chrono_fsi/sph/ChFsiSystemSPH.h"

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::fsi;

%}

// Undefine ChApi and other macros that otherwise SWIG gives a syntax error
#define CH_FSI_API
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

%shared_ptr(chrono::ChFrame<double>)
%shared_ptr(chrono::fsi::ChFsiSystem)
%shared_ptr(chrono::fsi::sph::ChFsiFluidSystemSPH)
%shared_ptr(chrono::fsi::sph::ChFsiSystemSPH)
%shared_ptr(chrono::fsi::sph::ChSphVisualizationVSG)
%shared_ptr(chrono::fsi::sph::ChSphVisualizationVSG::ParticleColorCallback)
%shared_ptr(chrono::fsi::sph::ParticlePressureColorCallback)
%shared_ptr(chrono::fsi::sph::ParticleVelocityColorCallback)
%shared_ptr(chrono::fsi::sph::ParticleDensityColorCallback)
%shared_ptr(chrono::fsi::sph::ParticleHeightColorCallback)

%include "chrono_swig/interface/core/ChClassFactory.i"
%include "chrono_swig/interface/core/ChVector2.i"
%include "chrono_swig/interface/core/ChVector3.i"
%include "chrono_swig/interface/core/ChMatrix.i"
%include "chrono_swig/interface/core/ChCoordsys.i"
%include "chrono_swig/interface/core/ChFrame.i"
%include "chrono_swig/interface/core/ChPhysicsItem.i"
%include "chrono_swig/interface/core/ChVisualMaterial.i"
%include "chrono_swig/interface/core/ChVisualShape.i"
%include "chrono_swig/interface/core/ChVisualModel.i"
%include "chrono_swig/interface/core/ChColor.i"
%include "chrono_swig/interface/core/ChColormap.i"
%include "chrono_swig/interface/core/ChBody.i"
%include "chrono_swig/interface/core/ChGeometry.i"
%include "chrono_swig/interface/core/ChBodyGeometry.i"
%include "chrono_swig/interface/core/ChParticleCloud.i"
#ifdef CHRONO_VSG
%include "chrono_swig/interface/vsg/ChVisualSystemVSG.i"
#endif

%import(module="pychrono.core") "chrono_swig/interface/core/ChSystem.i"

%include "ChFsiDefinitions.i"
%include "ChFsiDefinitionsSPH.i"
%include "ChFsiSystem.i"
%include "ChFsiFluidSystemSPH.i"
%include "ChFsiSystemSPH.i"
%include "ChFsiProblemSPH.i"
#ifdef CHRONO_VSG
%include "ChSphVisualizationVSG.i"
#endif
