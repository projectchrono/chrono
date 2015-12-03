#pragma once

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"

namespace chrono {
class CH_PARALLEL_API ChConstraintFluidFluid {
  public:
    ChConstraintFluidFluid() { data_manager = 0; }
    ~ChConstraintFluidFluid() {}
    void Setup(ChParallelDataManager* data_container_) {
        data_manager = data_container_;

        num_fluid_contacts = data_manager->num_fluid_contacts;
        num_fluid_bodies = data_manager->num_fluid_bodies;
        num_rigid_bodies = data_manager->num_rigid_bodies;
        num_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
        num_unilaterals = data_manager->num_unilaterals;
        num_bilaterals = data_manager->num_bilaterals;
        num_shafts = data_manager->num_shafts;

        index_offset = num_unilaterals + num_bilaterals + num_rigid_fluid_contacts * 3;
        body_offset = num_rigid_bodies * 6 + num_shafts;
    }
    void Build_D();
    void Build_b();
    void Build_E();
    // Based on the list of contacts figure out what each fluid particles
    // neighbors are
    void DetermineNeighbors();
    void Project(real* gamma);
    void GenerateSparsity();
    // generate the sparsity when treating the fluid as a rigid body
    void GenerateSparsityRigid();

    // generate the sparsity when treating the fluid as constraint fluid
    void GenerateSparsityFluid();
    // Fill in the jacobian based on fluid or rigid
    void Build_D_Rigid();
    void Build_D_Fluid();

    void Density_Fluid();
    void Normalize_Density_Fluid();
    void ArtificialPressure();

  protected:
    host_vector<real> dist_temp;

    host_vector<real3> viscosity_row_1, viscosity_row_2, viscosity_row_3;

    host_vector<real3> den_con;
    host_vector<real4> den_vec;

    //
    host_vector<M33> shear_tensor;
    host_vector<real> shear_trace;
    // Pointer to the system's data manager
    ChParallelDataManager* data_manager;

  private:
    uint num_fluid_contacts;
    uint num_fluid_bodies;
    uint num_rigid_bodies;
    uint num_rigid_fluid_contacts;
    uint num_unilaterals;
    uint num_bilaterals;
    uint num_shafts;
    uint index_offset;
    uint body_offset;
};
}
