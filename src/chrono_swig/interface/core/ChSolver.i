#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// MULTIPLE INHERITANCE WORKAROUND

// (B) Methods of a base class that SWIG discards that *are* overriden in ChSolver***

// Ensure that these functions are not marked as 'overrides' in the generated C# code.

%csmethodmodifiers chrono::ChIterativeSolverLS::Setup "public"

%csmethodmodifiers chrono::ChIterativeSolverLS::Solve "public"
%csmethodmodifiers chrono::ChSolverAPGD::Solve "public"
%csmethodmodifiers chrono::ChSolverBB::Solve "public"
%csmethodmodifiers chrono::ChSolverPJacobi::Solve "public"
%csmethodmodifiers chrono::ChSolverPSOR::Solve "public"
%csmethodmodifiers chrono::ChSolverBiCGSTAB::Solve "public"
%csmethodmodifiers chrono::ChSolverGMRES::Solve "public"
%csmethodmodifiers chrono::ChSolverMINRES::Solve "public"
%csmethodmodifiers chrono::ChSolverADMM::Solve "public"

%csmethodmodifiers chrono::ChSolverAPGD::GetType "public"
%csmethodmodifiers chrono::ChSolverBB::GetType "public"
%csmethodmodifiers chrono::ChSolverPJacobi::GetType "public"
%csmethodmodifiers chrono::ChSolverPSOR::GetType "public"
%csmethodmodifiers chrono::ChSolverBiCGSTAB::GetType "public"
%csmethodmodifiers chrono::ChSolverGMRES::GetType "public"
%csmethodmodifiers chrono::ChSolverMINRES::GetType "public"
%csmethodmodifiers chrono::ChSolverADMM::GetType "public"

%csmethodmodifiers chrono::ChSolverAPGD::GetError "public"
%csmethodmodifiers chrono::ChSolverBB::GetError "public"
%csmethodmodifiers chrono::ChSolverPJacobi::GetError "public"
%csmethodmodifiers chrono::ChSolverPSOR::GetError "public"
%csmethodmodifiers chrono::ChSolverBiCGSTAB::GetError "public"
%csmethodmodifiers chrono::ChSolverGMRES::GetError "public"
%csmethodmodifiers chrono::ChSolverMINRES::GetError "public"
%csmethodmodifiers chrono::ChSolverADMM::GetError "public"

%csmethodmodifiers chrono::ChIterativeSolverVI::GetIterations "public"
%csmethodmodifiers chrono::ChSolverBiCGSTAB::GetIterations "public"
%csmethodmodifiers chrono::ChSolverGMRES::GetIterations "public"
%csmethodmodifiers chrono::ChSolverMINRES::GetIterations "public"

%csmethodmodifiers chrono::ChSolverAPGD::ArchiveIn "public"
%csmethodmodifiers chrono::ChSolverBB::ArchiveIn "public"
%csmethodmodifiers chrono::ChSolverPJacobi::ArchiveIn "public"
%csmethodmodifiers chrono::ChSolverPSOR::ArchiveIn "public"
%csmethodmodifiers chrono::ChSolverBiCGSTAB::ArchiveIn "public"
%csmethodmodifiers chrono::ChSolverGMRES::ArchiveIn "public"
%csmethodmodifiers chrono::ChSolverMINRES::ArchiveIn "public"
%csmethodmodifiers chrono::ChSolverADMM::ArchiveIn "public"

%csmethodmodifiers chrono::ChSolverAPGD::ArchiveOut "public"
%csmethodmodifiers chrono::ChSolverBB::ArchiveOut "public"
%csmethodmodifiers chrono::ChSolverPJacobi::ArchiveOut "public"
%csmethodmodifiers chrono::ChSolverPSOR::ArchiveOut "public"
%csmethodmodifiers chrono::ChSolverBiCGSTAB::ArchiveOut "public"
%csmethodmodifiers chrono::ChSolverGMRES::ArchiveOut "public"
%csmethodmodifiers chrono::ChSolverMINRES::ArchiveOut "public"
%csmethodmodifiers chrono::ChSolverADMM::ArchiveOut "public"

#endif             // --------------------------------------------------------------------- CSHARP

%{
#include <cstdlib>
#include <cmath>

#include "chrono/solver/ChSolver.h"
#include "chrono/solver/ChSolverVI.h"
#include "chrono/solver/ChSolverLS.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChIterativeSolverVI.h"

#include "chrono/solver/ChSolverBB.h"
#include "chrono/solver/ChSolverAPGD.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/solver/ChSolverPJacobi.h"
#include "chrono/solver/ChSolverADMM.h"

using namespace chrono;
%}

%shared_ptr(chrono::ChSolver)
%shared_ptr(chrono::ChSolverVI)
%shared_ptr(chrono::ChSolverLS)
%shared_ptr(chrono::ChDirectSolverLS)
%shared_ptr(chrono::ChIterativeSolver)
%shared_ptr(chrono::ChIterativeSolverLS)
%shared_ptr(chrono::ChIterativeSolverVI)

%shared_ptr(chrono::ChSolverGMRES)
%shared_ptr(chrono::ChSolverBiCGSTAB)
%shared_ptr(chrono::ChSolverMINRES)
%shared_ptr(chrono::ChSolverBB)
%shared_ptr(chrono::ChSolverAPGD)
%shared_ptr(chrono::ChSolverPSOR)
%shared_ptr(chrono::ChSolverPJacobi)
%shared_ptr(chrono::ChSolverSparseLU)
%shared_ptr(chrono::ChSolverSparseQR)
%shared_ptr(chrono::ChSolverADMM)

// Parse the header file to generate wrappers
%include "../../../chrono/solver/ChSolver.h"
%include "../../../chrono/solver/ChSolverVI.h"
%include "../../../chrono/solver/ChSolverLS.h"
%include "../../../chrono/solver/ChDirectSolverLS.h"
%include "../../../chrono/solver/ChIterativeSolver.h"
%include "../../../chrono/solver/ChIterativeSolverLS.h"
%include "../../../chrono/solver/ChIterativeSolverVI.h"

%include "../../../chrono/solver/ChSolverBB.h"
%include "../../../chrono/solver/ChSolverAPGD.h"
%include "../../../chrono/solver/ChSolverPSOR.h"
%include "../../../chrono/solver/ChSolverPJacobi.h"
%include "../../../chrono/solver/ChSolverADMM.h"
