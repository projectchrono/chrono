#include "chrono_parallel/ChConfigParallel.h"
#include "ChSolverParallel.h"

namespace chrono {
class CH_PARALLEL_API ChSolverAPGD : public ChSolverParallel {
 public:

   ChSolverAPGD()
         :
           ChSolverParallel() {

      //APGD specific
      step_shrink = .9;
      step_grow = 2.0;
      init_theta_k = 1.0;

   }
   ~ChSolverAPGD() {

   }

   void Solve() {
      if (num_constraints == 0) {return;}
      total_iteration += SolveAPGD(max_iteration, num_constraints, data_container->host_data.rhs_data, data_container->host_data.gamma_data);
      current_iteration = total_iteration;
   }
   // Solve using the Accelerated Projected Gradient Descent Method
   uint SolveAPGD(
                  const uint max_iter,           // Maximum number of iterations
                  const uint size,               // Number of unknowns
                  const custom_vector<real> &b,  // Rhs vector
                  custom_vector<real> &x         // The vector of unknowns
                  );

   // Compute the residual for the solver
   // TODO: What is the best way to explain this...
   real Res4(
             const int SIZE,
             real* mg_tmp,
             const real* b,
             real*x,
             real* mb_tmp);

   // Set parameters for growing and shrinking the step size
   void SetAPGDParams(
                      real theta_k,
                      real shrink,
                      real grow);

   //APGD specific vectors
   custom_vector<real> obj2_temp, obj1_temp, ms, mg_tmp2, mb_tmp, mg_tmp, mg_tmp1, mg, ml, mx, my, ml_candidate;

   real init_theta_k;
   real step_shrink;
   real step_grow;

};
}
