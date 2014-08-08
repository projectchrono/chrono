#include "chrono_parallel/ChConfigParallel.h"
#include "ChSolverParallel.h"

namespace chrono {
class CH_PARALLEL_API ChSolverMinRes : public ChSolverParallel {
 public:

   ChSolverMinRes()
         :
           ChSolverParallel() {

   }
   ~ChSolverMinRes() {

   }

   void Solve() {
      if (num_constraints == 0) {
         return;
      }
      total_iteration += SolveMinRes(max_iteration, num_constraints, data_container->host_data.rhs_data, data_container->host_data.gamma_data);
      current_iteration = total_iteration;
   }
   // Solve using the Accelerated Projected Gradient Descent Method
   uint SolveMinRes(const uint max_iter,           // Maximum number of iterations
                    const uint size,               // Number of unknowns
                    const custom_vector<real> &b,  // Rhs vector
                    custom_vector<real> &x         // The vector of unknowns
                    );

   custom_vector<real> mr, ml, mp, mz, mNMr, mNp, mMNp, mtmp;
   custom_vector<real> mz_old;
   custom_vector<real> mNMr_old;

};
}
