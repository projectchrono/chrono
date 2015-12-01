#pragma once
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"
namespace chrono {
class CH_PARALLEL_API ChConstraintRigidFluid {
  public:
    ChConstraintRigidFluid() { data_manager = 0; }
    ~ChConstraintRigidFluid() {}
    void Setup(ChParallelDataManager* data_container_) { data_manager = data_container_; }
    void Build_D();
    void Build_b();
    void Build_E();
    void Project(real* gamma);
    void GenerateSparsity();

  protected:
    // Pointer to the system's data manager
    ChParallelDataManager* data_manager;
};
}
