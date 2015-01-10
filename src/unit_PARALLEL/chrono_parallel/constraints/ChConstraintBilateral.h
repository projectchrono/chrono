#ifndef CHCONSTRAINT_BILATERAL_H
#define CHCONSTRAINT_BILATERAL_H

#include "chrono_parallel/ChBaseParallel.h"

namespace chrono {
class CH_PARALLEL_API ChConstraintBilateral : public ChBaseParallel {
 public:
  ChConstraintBilateral() {}
  void Setup(ChParallelDataManager* data_container_) {
    data_container = data_container_;
    Initialize();
  }
  ~ChConstraintBilateral() {}

  //Compute the vector of corrections
  void Build_b();
  //Compute the diagonal compliance matrix
  void Build_E();
  //Compute the jacobian matrix, no allocation is performed here,
  //GenerateSparsity should take care of that
  void Build_D();

  // This function will fill in the non zero entries in the bilateral jacobian
  // with one. This operation is sequential
  void GenerateSparsity(SOLVERMODE solver_mode);
};
}

#endif
