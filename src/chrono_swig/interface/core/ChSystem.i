%extend chrono::ChSystem 
{
// Allow serialization of System without wrapping of ChArchive
void SerializeToJSON(std::string path) {
  chrono::ChStreamOutAsciiFile mfileo(path.c_str());
  chrono::ChArchiveOutJSON marchiveout(mfileo);
  marchiveout << chrono::CHNVP(*$self, "sys");
}

}



#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// MULTIPLE INHERITANCE WORKAROUND

// Extend ChSystem with SetSolver functions that accept concrete solver types
%extend chrono::ChSystem
{
void SetSolver(std::shared_ptr<ChSolverPSOR> solver)     {$self->SetSolver(std::static_pointer_cast<ChSolver>(solver));}
void SetSolver(std::shared_ptr<ChSolverPJacobi> solver)  {$self->SetSolver(std::static_pointer_cast<ChSolver>(solver));}
void SetSolver(std::shared_ptr<ChSolverBB> solver)       {$self->SetSolver(std::static_pointer_cast<ChSolver>(solver));}
void SetSolver(std::shared_ptr<ChSolverAPGD> solver)     {$self->SetSolver(std::static_pointer_cast<ChSolver>(solver));}
void SetSolver(std::shared_ptr<ChSolverSparseLU> solver) {$self->SetSolver(std::static_pointer_cast<ChSolver>(solver));}
void SetSolver(std::shared_ptr<ChSolverSparseQR> solver) {$self->SetSolver(std::static_pointer_cast<ChSolver>(solver));}
void SetSolver(std::shared_ptr<ChSolverGMRES> solver)    {$self->SetSolver(std::static_pointer_cast<ChSolver>(solver));}
void SetSolver(std::shared_ptr<ChSolverBiCGSTAB> solver) {$self->SetSolver(std::static_pointer_cast<ChSolver>(solver));}
void SetSolver(std::shared_ptr<ChSolverMINRES> solver)   {$self->SetSolver(std::static_pointer_cast<ChSolver>(solver));}
}

#endif             // --------------------------------------------------------------------- CSHARP

%{
#include "chrono/physics/ChSystem.h"
#include "chrono/timestepper/ChIntegrable.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/timestepper/ChTimestepperHHT.h"

using namespace chrono;

%}

namespace chrono { 
class ChVisualSystem; 
}

%shared_ptr(chrono::ChSystem)
%shared_ptr(chrono::ChSystem::CustomCollisionCallback)

// Forward ref
%import "chrono_swig/interface/core/ChAssembly.i"
%import "chrono_swig/interface/core/ChTimestepper.i"
//%import "chrono_swig/interface/core/ChSolver.i"
%import "chrono_swig/interface/core/ChCollisionModel.i"
%import "chrono_swig/interface/core/ChCollisionInfo.i"

// Cross-inheritance for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.
%feature("director") CustomCollisionCallback;

// Parse the header file to generate wrappers
%include "../../../chrono/physics/ChSystem.h" 





