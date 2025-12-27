// GetType hides System.Object.GetType - use 'new' keyword in C#
#ifdef SWIGCSHARP
%csmethodmodifiers chrono::ChTimestepper::GetType "public new virtual"
#endif

%{
#include <cmath>
#include <cstdlib>
#include <memory>
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
#include "chrono/physics/ChSystem.h"

using namespace chrono;
%}

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
%shared_ptr(chrono::ChStaticNonLinearRheonomicAnalysis::IterationCallback)
%shared_ptr(chrono::ChStaticNonLinearIncremental)
%shared_ptr(chrono::ChStaticNonLinearIncremental::LoadIncrementCallback)

%feature("director") chrono::ChStaticNonLinearRheonomicAnalysis::IterationCallback;
%rename("ChStaticNonLinearRheonomicAnalysis_IterationCallback") chrono::ChStaticNonLinearRheonomicAnalysis::IterationCallback;

%feature("director") chrono::ChStaticNonLinearIncremental::LoadIncrementCallback;
%rename("ChStaticNonLinearIncremental_LoadIncrementCallback") chrono::ChStaticNonLinearIncremental::LoadIncrementCallback;

// C++ implicit timesteppers use multiple inheritance:
//   class ChTimestepperEulerImplicit : public ChTimestepperIIorder, public ChTimestepperImplicit
// 
// ChTimestepperIIorder does NOT inherit from ChTimestepper
// ChTimestepperImplicit DOES inherit from ChTimestepper
// 
#ifdef SWIGCSHARP
// Ensure ChSystem-derived integrables pass the actual system pointer into native code - detects when the
// integrable is actually a ChSystem and returns ChSystem.getCPtr(sys) instead of the upcast base pointer that SWIG is defaulting to
%typemap(cscode) chrono::ChIntegrableIIorder %{
  internal static global::System.Runtime.InteropServices.HandleRef GetSystemAwareCPtr(ChIntegrableIIorder obj) {
    if (obj == null) {
      return new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
    }
    ChSystem sys = obj as ChSystem;
    if (sys != null) {
      return ChSystem.getCPtr(sys);
    }
    return ChIntegrableIIorder.getCPtr(obj);
  }
%}

%typemap(csin) chrono::ChIntegrableIIorder * "ChIntegrableIIorder.GetSystemAwareCPtr($csinput)"
%typemap(csin) const chrono::ChIntegrableIIorder * "ChIntegrableIIorder.GetSystemAwareCPtr($csinput)"

// Helper template to create C# accessibility for the implicit base shared_ptr conversions
%define DECLARE_IMPLICIT_TIMESTEPPER_UPCAST(CLASSNAME)
%inline %{
std::shared_ptr<chrono::ChTimestepperImplicit>* CLASSNAME##_to_ChTimestepperImplicit(
    std::shared_ptr<chrono::CLASSNAME>* ptr) {
    if (!ptr)
        return nullptr;
    return new std::shared_ptr<chrono::ChTimestepperImplicit>(
        std::static_pointer_cast<chrono::ChTimestepperImplicit>(*ptr));
}
%}
%enddef

// Macro to fix the base class and constructor body for implicit timesteppers
%define FIX_IMPLICIT_TIMESTEPPER_INHERITANCE(CLASSNAME)
DECLARE_IMPLICIT_TIMESTEPPER_UPCAST(CLASSNAME)
%typemap(csbase, replace="1") chrono::CLASSNAME "ChTimestepperImplicit"
%typemap(csbody) chrono::CLASSNAME %{
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnDerived;

  internal $csclassname(global::System.IntPtr cPtr, bool cMemoryOwn) : base(chronoPINVOKE.$csclassname_to_ChTimestepperImplicit(new global::System.Runtime.InteropServices.HandleRef(null, cPtr)), true) {
    swigCMemOwnDerived = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr($csclassname obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwnDerived) {
          swigCMemOwnDerived = false;
          chronoPINVOKE.delete_$csclassname(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }
%}
%typemap(csdispose) chrono::CLASSNAME ""
%typemap(csdisposing) chrono::CLASSNAME ""
%typemap(csdisposing_derived) chrono::CLASSNAME ""
%enddef

FIX_IMPLICIT_TIMESTEPPER_INHERITANCE(ChTimestepperEulerImplicit)
FIX_IMPLICIT_TIMESTEPPER_INHERITANCE(ChTimestepperEulerImplicitLinearized)
FIX_IMPLICIT_TIMESTEPPER_INHERITANCE(ChTimestepperEulerImplicitProjected)
FIX_IMPLICIT_TIMESTEPPER_INHERITANCE(ChTimestepperTrapezoidal)
FIX_IMPLICIT_TIMESTEPPER_INHERITANCE(ChTimestepperTrapezoidalLinearized)
FIX_IMPLICIT_TIMESTEPPER_INHERITANCE(ChTimestepperNewmark)
FIX_IMPLICIT_TIMESTEPPER_INHERITANCE(ChTimestepperHHT)

#endif // SWIGCSHARP

%include "../../../chrono/timestepper/ChState.h"
%include "../../../chrono/timestepper/ChIntegrable.h"
%include "../../../chrono/timestepper/ChTimestepper.h"
%include "../../../chrono/timestepper/ChTimestepperImplicit.h"
%include "../../../chrono/timestepper/ChTimestepperHHT.h"
%include "../../../chrono/timestepper/ChAssemblyAnalysis.h"
%include "../../../chrono/timestepper/ChStaticAnalysis.h"
