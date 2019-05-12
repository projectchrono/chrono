%{

/* Includes the header in the wrapper code */
#include <cstdlib>
#include <cmath>
#include <cstdlib>
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMath.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/serialization/ChArchive.h"
#include "chrono/timestepper/ChIntegrable.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/timestepper/ChTimestepperHHT.h"
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMath.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/timestepper/ChIntegrable.h"

using namespace chrono;
%}

// Tell SWIG about parent class in Python
%import "ChVectorDynamic.i"

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
  
%include "../chrono/timestepper/ChState.h"
%include "../chrono/timestepper/ChIntegrable.h"
%include "../chrono/timestepper/ChTimestepper.h"
%include "../chrono/timestepper/ChTimestepperHHT.h"






