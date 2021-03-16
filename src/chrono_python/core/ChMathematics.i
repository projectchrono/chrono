%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChDistribution.h"
using namespace chrono;

%}

%shared_ptr(chrono::ChDistribution)
%shared_ptr(chrono::ChConstantDistribution)
%shared_ptr(chrono::ChContinuumDistribution)
%shared_ptr(chrono::ChDiscreteDistribution)
%shared_ptr(chrono::ChMinMaxDistribution)
%shared_ptr(chrono::ChNormalDistribution)
%shared_ptr(chrono::ChWeibullDistribution)
%shared_ptr(chrono::ChZhangDistribution)

/* Parse the header file to generate wrappers */
%include "../../chrono/core/ChMathematics.h"    
%include "../../chrono/core/ChDistribution.h"    
