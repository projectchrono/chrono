// GetType hides System.Object.GetType - use 'new' keyword in C#
#ifdef SWIGCSHARP
%csmethodmodifiers chrono::ChTimestepper::GetType "public new virtual"
#endif

%{
#include <cmath>
#include <cstdlib>
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/serialization/ChArchive.h"
#include "chrono/timestepper/ChIntegrable.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/timestepper/ChTimestepperImplicit.h"
#include "chrono/timestepper/ChTimestepperHHT.h"
#include "chrono/timestepper/ChAssemblyAnalysis.h"
#include "chrono/timestepper/ChStaticAnalysis.h"

using namespace chrono;
%}

// Tell SWIG about parent class in Python

/* Parse the header file to generate wrappers */

%shared_ptr(chrono::ChIntegrable)
%shared_ptr(chrono::ChIntegrableIIorder)

%shared_ptr(chrono::ChTimestepper)
%shared_ptr(chrono::ChTimestepperIorder)
%shared_ptr(chrono::ChTimestepperIIorder)

%shared_ptr(chrono::ChTimestepperExplicit)  
%shared_ptr(chrono::ChTimestepperEulerExplicitIorder)
%shared_ptr(chrono::ChTimestepperEulerExplicitIIorder)
%shared_ptr(chrono::ChTimestepperEulerSemiImplicit)
%shared_ptr(chrono::ChTimestepperRungeKutta)
%shared_ptr(chrono::ChTimestepperHeun)
%shared_ptr(chrono::ChTimestepperLeapfrog)

%shared_ptr(chrono::ChTimestepperImplicit)
%shared_ptr(chrono::ChTimestepperEulerImplicit)
%shared_ptr(chrono::ChTimestepperEulerImplicitLinearized)
%shared_ptr(chrono::ChTimestepperEulerImplicitProjected)
%shared_ptr(chrono::ChTimestepperTrapezoidal)
%shared_ptr(chrono::ChTimestepperTrapezoidalLinearized)
%shared_ptr(chrono::ChTimestepperNewmark)
%shared_ptr(chrono::ChTimestepperHHT)

%shared_ptr(chrono::ChAssemblyAnalysis)  
%shared_ptr(chrono::ChStaticAnalysis)
%shared_ptr(chrono::ChStaticLinearAnalysis)
%shared_ptr(chrono::ChStaticNonLinearAnalysis)
%shared_ptr(chrono::ChStaticNonLinearRheonomicAnalysis)
%shared_ptr(chrono::ChStaticNonLinearIncremental)

// Special handling: C++ implicit timesteppers inherit from both ChTimestepperIIorder and ChTimestepperImplicit
// ChTimestepperIIorder does NOT inherit from ChTimestepper, but ChTimestepperImplicit does but the problem is
// SWIG picks the first base (ChTimestepperIIorder), breaking any expecting ChTimestepper
// These typemaps must come after the %shared_ptr declarations and before includes
//
// shared_ptr in C++ (held by ChSystem) manages the lifetime so we setting swigCMemOwnDerived
// to false to stop C# from trying to delete the c++ object
// instead - just let the chsystem handle the timestepper on c++ side, and c# side just clears the handle at dispose

#ifdef SWIGCSHARP

// Force correct base class for the implicit timesteppers
%typemap(csbase, replace="1") chrono::ChTimestepperEulerImplicit "ChTimestepperImplicit"
%typemap(csbase, replace="1") chrono::ChTimestepperEulerImplicitLinearized "ChTimestepperImplicit"
%typemap(csbase, replace="1") chrono::ChTimestepperEulerImplicitProjected "ChTimestepperImplicit"
%typemap(csbase, replace="1") chrono::ChTimestepperTrapezoidal "ChTimestepperImplicit"
%typemap(csbase, replace="1") chrono::ChTimestepperTrapezoidalLinearized "ChTimestepperImplicit"
%typemap(csbase, replace="1") chrono::ChTimestepperNewmark "ChTimestepperImplicit"
%typemap(csbase, replace="1") chrono::ChTimestepperHHT "ChTimestepperImplicit"

// Macro to define complete typemap set for implicit timesteppers with forced base class
// (ince we're forcing a different base class with csbase, we must override all dispose-related typemaps(
// But since these are shared_ptr wrapped objects - C# must not try to garbage colleect/delete them
// c++ shared_ptr (in ChSystem) manages the lifetime and handles deletion (csharp does not own the memory!)
%define IMPLICIT_TIMESTEPPER_CSHARP_TYPEMAPS(CPPTYPE, CSTYPE)

// Body with derived pattern - calls base constructor with pointer
// Note: No swigCMemOwn tracking needed - shared_ptr in C++ manages lifetime, not C#
%typemap(csbody) CPPTYPE %{
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;

  internal $csclassname(global::System.IntPtr cPtr, bool cMemoryOwn) : base(cPtr, false) {
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr($csclassname obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }
%}

// Suppress the default csdispose
%typemap(csdispose) CPPTYPE ""
// Suppress the default csdisposing
%typemap(csdisposing, methodname="Dispose", methodmodifiers="protected override", parameters="bool disposing") CPPTYPE ""

// Since shared_ptr owns the memory just clear the handle (avoid GB collection issues if any)
%typemap(cscode) CPPTYPE %{
  ~$csclassname() {
    Dispose(false);
  }

  public new void Dispose() {
    Dispose(true);
    global::System.GC.SuppressFinalize(this);
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        // Do not delete - C++ owns the memory (since set up with a chrono_typres::make_shared)
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }
%}

%enddef

// Apply the typemaps to all the implicit timesteppers
IMPLICIT_TIMESTEPPER_CSHARP_TYPEMAPS(chrono::ChTimestepperEulerImplicit, ChTimestepperEulerImplicit)
IMPLICIT_TIMESTEPPER_CSHARP_TYPEMAPS(chrono::ChTimestepperEulerImplicitLinearized, ChTimestepperEulerImplicitLinearized)
IMPLICIT_TIMESTEPPER_CSHARP_TYPEMAPS(chrono::ChTimestepperEulerImplicitProjected, ChTimestepperEulerImplicitProjected)
IMPLICIT_TIMESTEPPER_CSHARP_TYPEMAPS(chrono::ChTimestepperTrapezoidal, ChTimestepperTrapezoidal)
IMPLICIT_TIMESTEPPER_CSHARP_TYPEMAPS(chrono::ChTimestepperTrapezoidalLinearized, ChTimestepperTrapezoidalLinearized)
IMPLICIT_TIMESTEPPER_CSHARP_TYPEMAPS(chrono::ChTimestepperNewmark, ChTimestepperNewmark)
IMPLICIT_TIMESTEPPER_CSHARP_TYPEMAPS(chrono::ChTimestepperHHT, ChTimestepperHHT)

#endif // SWIGCSHARP

%include "../../../chrono/timestepper/ChState.h"
%include "../../../chrono/timestepper/ChIntegrable.h"
%include "../../../chrono/timestepper/ChTimestepper.h"
%include "../../../chrono/timestepper/ChTimestepperImplicit.h"
%include "../../../chrono/timestepper/ChTimestepperHHT.h"
%include "../../../chrono/timestepper/ChAssemblyAnalysis.h"
%include "../../../chrono/timestepper/ChStaticAnalysis.h"
