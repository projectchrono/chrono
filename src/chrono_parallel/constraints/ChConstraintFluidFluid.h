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

        // data_manager->host_data.fluid_contact_index.resize(num_fluid_bodies * max_neighbors);
        //        int cnt = 0;
        //        for (int p = 0; p < num_fluid_bodies; p++) {
        //            for (int i = 0; i < data_manager->c_counts_fluid_fluid[p]; i++) {
        //                data_manager->host_data.fluid_contact_index[p * max_neighbors + i] = cnt;
        //                cnt++;
        //            }
        //        }

        //        xij_dist_fluid_fluid.resize(num_fluid_bodies * max_neighbors);
        //        for (int a = 0; a < num_fluid_bodies; a++) {
        //            int p = data_manager->host_data.particle_indices_fluid[a];
        //            real3 pos_p = data_manager->host_data.pos_fluid[p];
        //            for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[a]; i++) {
        //                int b = data_manager->host_data.neighbor_fluid_fluid[a * max_neighbors + i];
        //                int q = data_manager->host_data.particle_indices_fluid[b];
        //                real3 xij = pos_p - data_manager->host_data.pos_fluid[q];
        //                xij_dist_fluid_fluid[a * max_neighbors + i] = real4(xij, Length(xij));
        //            }
        //        }
    }
    void Build_D();
    void Build_b();
    void Build_E();
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
    void Dx(const DynamicVector<real>& x, DynamicVector<real>& output);
    void D_Tx(const DynamicVector<real>& x, DynamicVector<real>& output);

  protected:
    // custom_vector<int> fluid_contact_idA, fluid_contact_idA_start;
    // custom_vector<int> fluid_contact_idB, fluid_start_index;
    // custom_vector<real> dist_temp;

    // custom_vector<real3> viscosity_row_1, viscosity_row_2, viscosity_row_3;
    // custom_vector<real4> xij_dist_fluid_fluid;
    int last_body;

    // custom_vector<real3> den_con;
    // custom_vector<real4> den_vec;

    //
    custom_vector<Mat33> shear_tensor;
    custom_vector<real> shear_trace;
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
