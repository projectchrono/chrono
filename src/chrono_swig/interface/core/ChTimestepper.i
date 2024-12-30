#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// MULTIPLE INHERITANCE WORKAROUND

// (A) Methods inherited from base classes that SWIG discards
//     (i.e. methods that *are not* overriden in ChTimestepperHHT and ChTimestepperEulerImplicit)

// First, ensure that these functions are not marked as 'overrides' in the generated C# code.

%csmethodmodifiers chrono::ChTimestepperHHT::SetMaxIters "public"
%csmethodmodifiers chrono::ChTimestepperHHT::SetRelTolerance "public"
%csmethodmodifiers chrono::ChTimestepperHHT::SetAbsTolerances "public"
%csmethodmodifiers chrono::ChTimestepperHHT::GetNumIterations "public"
%csmethodmodifiers chrono::ChTimestepperHHT::GetNumSetupCalls "public"
%csmethodmodifiers chrono::ChTimestepperHHT::GetNumSolveCalls "public"

%csmethodmodifiers chrono::ChTimestepperEulerImplicit::SetMaxIters "public"
%csmethodmodifiers chrono::ChTimestepperEulerImplicit::SetRelTolerance "public"
%csmethodmodifiers chrono::ChTimestepperEulerImplicit::SetAbsTolerances "public"
%csmethodmodifiers chrono::ChTimestepperEulerImplicit::GetNumIterations "public"
%csmethodmodifiers chrono::ChTimestepperEulerImplicit::GetNumSetupCalls "public"
%csmethodmodifiers chrono::ChTimestepperEulerImplicit::GetNumSolveCalls "public"

%csmethodmodifiers chrono::ChTimestepper::GetType "public virtual new"


// Second, extend ChTimestepperHHT and ChTimestepperEulerImplicit with implementations of these functions

%extend chrono::ChTimestepperHHT
{
    void SetMaxIters(int iters)                             {$self->SetMaxIters(iters);}
    void SetRelTolerance(double rel_tol)                    {$self->SetRelTolerance(rel_tol);}
    void SetAbsTolerances(double abs_tolS, double abs_tolL) {$self->SetAbsTolerances(abs_tolS, abs_tolL);}
    void SetAbsTolerances(double abs_tol)                   {$self->SetAbsTolerances(abs_tol);}
    int GetNumIterations() const {return $self->GetNumIterations();}
    int GetNumSetupCalls() const {return $self->GetNumSetupCalls();}
    int GetNumSolveCalls() const {return $self->GetNumSolveCalls();}
}

%extend chrono::ChTimestepperEulerImplicit
{
    void SetMaxIters(int iters)                             {$self->SetMaxIters(iters);}
    void SetRelTolerance(double rel_tol)                    {$self->SetRelTolerance(rel_tol);}
    void SetAbsTolerances(double abs_tolS, double abs_tolL) {$self->SetAbsTolerances(abs_tolS, abs_tolL);}
    void SetAbsTolerances(double abs_tol)                   {$self->SetAbsTolerances(abs_tol);}
    int GetNumIterations() const {return $self->GetNumIterations();}
    int GetNumSetupCalls() const {return $self->GetNumSetupCalls();}
    int GetNumSolveCalls() const {return $self->GetNumSolveCalls();}
}

#endif             // --------------------------------------------------------------------- CSHARP

%{
#include <cmath>
#include <cstdlib>
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/serialization/ChArchive.h"
#include "chrono/timestepper/ChIntegrable.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/timestepper/ChTimestepper.h"
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
%shared_ptr(chrono::ChTimestepperEulerExpl)
%shared_ptr(chrono::ChTimestepperEulerExplIIorder)
%shared_ptr(chrono::ChTimestepperEulerSemiImplicit)
%shared_ptr(chrono::ChTimestepperRungeKuttaExpl)
%shared_ptr(chrono::ChTimestepperHeun)
%shared_ptr(chrono::ChTimestepperLeapfrog)
%shared_ptr(chrono::ChTimestepperEulerImplicit)
%shared_ptr(chrono::ChTimestepperEulerImplicitLinearized)
%shared_ptr(chrono::ChTimestepperEulerImplicitProjected)
%shared_ptr(chrono::ChTimestepperTrapezoidalLinearized)
%shared_ptr(chrono::ChTimestepperTrapezoidalLinearized2)
%shared_ptr(chrono::ChTimestepperTrapezoidal)
%shared_ptr(chrono::ChTimestepperNewmark)
%shared_ptr(chrono::ChTimestepperHHT)
%shared_ptr(chrono::ChImplicitIterativeTimestepper)
%shared_ptr(chrono::ChImplicitTimestepper)
%shared_ptr(chrono::ChExplicitTimestepper)  
%shared_ptr(chrono::ChAssemblyAnalysis)  
%shared_ptr(chrono::ChStaticAnalysis)
%shared_ptr(chrono::ChStaticLinearAnalysis)
%shared_ptr(chrono::ChStaticNonLinearAnalysis)
%shared_ptr(chrono::ChStaticNonLinearRheonomicAnalysis)
%shared_ptr(chrono::ChStaticNonLinearIncremental)

%include "../../../chrono/timestepper/ChState.h"
%include "../../../chrono/timestepper/ChIntegrable.h"
%include "../../../chrono/timestepper/ChTimestepper.h"
%include "../../../chrono/timestepper/ChTimestepperHHT.h"
%include "../../../chrono/timestepper/ChAssemblyAnalysis.h"
%include "../../../chrono/timestepper/ChStaticAnalysis.h"