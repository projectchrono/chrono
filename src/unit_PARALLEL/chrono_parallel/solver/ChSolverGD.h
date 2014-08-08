#include "chrono_parallel/ChConfigParallel.h"
#include "ChSolverParallel.h"

namespace chrono {
class CH_PARALLEL_API ChSolverGD : public ChSolverParallel {
 public:

   ChSolverGD()
         :
           ChSolverParallel() {

   }
   ~ChSolverGD() {

   }

   void Solve() {
      if (num_constraints == 0) {return;}
      total_iteration += SolveGD(max_iteration, num_constraints, data_container->host_data.rhs_data, data_container->host_data.gamma_data);
      current_iteration = total_iteration;
   }
   // Solve using the Accelerated Projected Gradient Descent Method
   uint SolveGD(
                const uint max_iter,           // Maximum number of iterations
                const uint size,               // Number of unknowns
                const custom_vector<real> &b,  // Rhs vector
                custom_vector<real> &x         // The vector of unknowns
                );

};
}
