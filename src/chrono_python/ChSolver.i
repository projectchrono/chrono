%{

/* Includes the header in the wrapper code */
#include <cstdlib>
#include <cmath>
#include "chrono/solver/ChSolver.h"
#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/solver/ChSolverMINRES.h"

using namespace chrono;
%}

// Tell SWIG about parent class in Python

/* Parse the header file to generate wrappers */

%shared_ptr(chrono::ChSolver)
%shared_ptr(chrono::ChIterativeSolver)
%shared_ptr(chrono::ChSolverMINRES)
  
%include "../chrono/solver/ChSolver.h"
%include "../chrono/solver/ChIterativeSolver.h"
%ignore chrono::ChSolverMINRES::GetType;
%include "../chrono/solver/ChSolverMINRES.h"






