%extend chrono::ChSystem 
{
// Allow serialization of System without wrapping of ChArchive
void SerializeToJSON(std::string path) {
  std::ofstream mfileo(path.c_str());
  chrono::ChArchiveOutJSON archive_out(mfileo);
  archive_out << chrono::CHNVP(*$self, "sys");
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

// SetTimestepper overloads for implicit timesteppers (multiple inheritance workaround) use dynamic_pointer_cast for the multi-inheritance robustness
// Recommend to use settimesteppertype() and 'gettimestepper() as XXXX' instead where possible to create new timesteppers
%extend chrono::ChSystem
{
void SetTimestepper(std::shared_ptr<ChTimestepperEulerImplicit> stepper)           {$self->SetTimestepper(std::dynamic_pointer_cast<ChTimestepper>(stepper));}
void SetTimestepper(std::shared_ptr<ChTimestepperEulerImplicitLinearized> stepper) {$self->SetTimestepper(std::dynamic_pointer_cast<ChTimestepper>(stepper));}
void SetTimestepper(std::shared_ptr<ChTimestepperEulerImplicitProjected> stepper)  {$self->SetTimestepper(std::dynamic_pointer_cast<ChTimestepper>(stepper));}
void SetTimestepper(std::shared_ptr<ChTimestepperTrapezoidal> stepper)             {$self->SetTimestepper(std::dynamic_pointer_cast<ChTimestepper>(stepper));}
void SetTimestepper(std::shared_ptr<ChTimestepperTrapezoidalLinearized> stepper)   {$self->SetTimestepper(std::dynamic_pointer_cast<ChTimestepper>(stepper));}
void SetTimestepper(std::shared_ptr<ChTimestepperNewmark> stepper)                 {$self->SetTimestepper(std::dynamic_pointer_cast<ChTimestepper>(stepper));}
void SetTimestepper(std::shared_ptr<ChTimestepperHHT> stepper)                     {$self->SetTimestepper(std::dynamic_pointer_cast<ChTimestepper>(stepper));}
}

#endif             // --------------------------------------------------------------------- CSHARP

%{
#include "chrono/physics/ChSystem.h"
#include "chrono/timestepper/ChIntegrable.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/timestepper/ChTimestepperImplicit.h"
#include "chrono/timestepper/ChTimestepperHHT.h"

using namespace chrono;

%}

namespace chrono { 
class ChVisualSystem; 
}

%shared_ptr(chrono::ChSystem)
%shared_ptr(chrono::ChSystem::CustomCollisionCallback)

%template(vector_ChSystem) std::vector< chrono::ChSystem* >;

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





