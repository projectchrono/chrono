#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// MULTIPLE INHERITANCE WORKAROUND
// (B) Methods of a base class that SWIG discards that *are* overriden in ChSolver***
// Ensure that these functions are not marked as 'overrides' in the generated C# code.

%csmethodmodifiers chrono::ChIterativeSolverLS::Setup "public virtual"
%csmethodmodifiers chrono::ChIterativeSolverVI::Setup "public virtual"

%csmethodmodifiers chrono::ChIterativeSolverLS::Solve "public virtual"
%csmethodmodifiers chrono::ChIterativeSolverVI::Solve "public virtual"

%csmethodmodifiers chrono::ChIterativeSolverLS::ArchiveIn "public virtual"
%csmethodmodifiers chrono::ChIterativeSolverVI::ArchiveIn "public virtual"

%csmethodmodifiers chrono::ChIterativeSolverLS::ArchiveOut "public virtual"
%csmethodmodifiers chrono::ChIterativeSolverVI::ArchiveOut "public virtual"

%csmethodmodifiers chrono::ChSolver::GetType "public virtual new"
%csmethodmodifiers chrono::ChIterativeSolverLS::GetType "public virtual new"
%csmethodmodifiers chrono::ChIterativeSolverVI::GetType "public virtual new"

%csmethodmodifiers chrono::ChSolverADMM::ArchiveOut "public override"
%csmethodmodifiers chrono::ChSolverBB::ArchiveOut "public override"

%csmethodmodifiers chrono::ChSolverADMM::ArchiveIn "public override"
%csmethodmodifiers chrono::ChSolverBB::ArchiveIn "public override"


// Methods inherited from ChSolver; they are re-created here where the multiple
// inheritance happens, so that derived classes can simply override them

%extend chrono::ChIterativeSolverLS 
{
Type GetType() const { return chrono::ChSolver::Type::CUSTOM; }
};

%extend chrono::ChIterativeSolverVI 
{
Type GetType() const { return chrono::ChSolver::Type::CUSTOM; }
};




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


%DefSharedPtrDynamicDowncast(chrono, ChSolver, ChDirectSolverLS)
%DefSharedPtrDynamicDowncast(chrono, ChSolver, ChIterativeSolverLS)
%DefSharedPtrDynamicDowncast(chrono, ChSolver, ChIterativeSolverVI)


%DefSharedPtrDynamicDowncast(chrono, ChIterativeSolverVI, ChSolverADMM)
%DefSharedPtrDynamicDowncast(chrono, ChIterativeSolverVI, ChSolverAPGD)
%DefSharedPtrDynamicDowncast(chrono, ChIterativeSolverVI, ChSolverBB)
%DefSharedPtrDynamicDowncast(chrono, ChIterativeSolverVI, ChSolverPSOR)

%DefSharedPtrDynamicDowncast(chrono, ChIterativeSolverLS, ChSolverGMRES)
%DefSharedPtrDynamicDowncast(chrono, ChIterativeSolverLS, ChSolverMINRES)
%DefSharedPtrDynamicDowncast(chrono, ChIterativeSolverLS, ChSolverBiCGSTAB)

%DefSharedPtrDynamicDowncast(chrono, ChDirectSolverLS, ChSolverSparseQR)
%DefSharedPtrDynamicDowncast(chrono, ChDirectSolverLS, ChSolverSparseLU)
