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
    void Initialize();
    void Build_D();
    void Build_b();
    void Build_E();
    void Project(real* gamma);
    void GenerateSparsity();

    void Density_Fluid();
    void Normalize_Density_Fluid();
    void ArtificialPressure();
    void XSPHViscosity();
    void Dx(const DynamicVector<real>& x, DynamicVector<real>& output);
    void D_Tx(const DynamicVector<real>& x, DynamicVector<real>& output);

  protected:
    custom_vector<Mat33> shear_tensor;
    custom_vector<real> shear_trace;
    // Pointer to the system's data manager
    ChParallelDataManager* data_manager;
    // custom_vector<real3> den_con_diag;
    custom_vector<real3> den_con_jac;

    custom_vector<real3> visc1_jac;
    custom_vector<real3> visc2_jac;
    custom_vector<real3> visc3_jac;
    custom_vector<real> density;

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
