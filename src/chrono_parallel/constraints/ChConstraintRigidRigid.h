// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: This class handles rigid contact and computes corrections
// and jacobians
// =============================================================================

#pragma once

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"

namespace chrono {

class CH_PARALLEL_API ChConstraintRigidRigid {
  public:
    ChConstraintRigidRigid() {
        data_manager = 0;
        offset = 3;
        inv_h = inv_hpa = inv_hhpa = 0;
    }

    ~ChConstraintRigidRigid() {}

    void Setup(ChParallelDataManager* data_container_) {
        data_manager = data_container_;
        uint num_contacts = data_manager->num_rigid_contacts;
        inv_h = 1 / data_manager->settings.step_size;
        inv_hpa = 1 / (data_manager->settings.step_size + data_manager->settings.solver.alpha);
        inv_hhpa = inv_h * inv_hpa;

        if (num_contacts > 0) {
            contact_active_pairs.resize(int(num_contacts));
            data_manager->host_data.coh_rigid_rigid.resize(num_contacts);
            data_manager->host_data.fric_rigid_rigid.resize(num_contacts);
            rotated_point_a.resize(num_contacts);
            rotated_point_b.resize(num_contacts);
            quat_a.resize(num_contacts);
            quat_b.resize(num_contacts);

#pragma omp parallel for
            for (int i = 0; i < (signed)num_contacts; i++) {
                vec2 body = data_manager->host_data.bids_rigid_rigid[i];
                uint b1 = body.x;
                uint b2 = body.y;

                contact_active_pairs[i] =
                    bool2(data_manager->host_data.active_rigid[b1] != 0, data_manager->host_data.active_rigid[b2] != 0);

                ////real coh = Max(
                ////    (data_manager->host_data.cohesion_data[b1] + data_manager->host_data.cohesion_data[b2]) * .5, 0.0);
                real coh = Min(data_manager->host_data.cohesion_data[b1], data_manager->host_data.cohesion_data[b2]);
                data_manager->host_data.coh_rigid_rigid[i] = coh;

                real3 f_a = data_manager->host_data.fric_data[b1];
                real3 f_b = data_manager->host_data.fric_data[b2];
                real3 mu;

                mu.x = (f_a.x == 0 || f_b.x == 0) ? 0 : (f_a.x + f_b.x) * .5;
                ////mu.y = (f_a.y == 0 || f_b.y == 0) ? 0 : (f_a.y + f_b.y) * .5;
                ////mu.z = (f_a.z == 0 || f_b.z == 0) ? 0 : (f_a.z + f_b.z) * .5;
                mu.y = Min(f_a.y, f_b.y);  // rolling
                mu.z = Min(f_a.z, f_b.z);  // spinning

                data_manager->host_data.fric_rigid_rigid[i] = mu;

                {
                    quaternion quaternion_conjugate = ~data_manager->host_data.rot_rigid[b1];
                    real3 sbar =
                        Rotate(data_manager->host_data.cpta_rigid_rigid[i] - data_manager->host_data.pos_rigid[b1],
                               quaternion_conjugate);

                    rotated_point_a[i] = real3_int(sbar, b1);
                    quat_a[i] = quaternion_conjugate;
                }
                {
                    quaternion quaternion_conjugate = ~data_manager->host_data.rot_rigid[b2];
                    real3 sbar =
                        Rotate(data_manager->host_data.cptb_rigid_rigid[i] - data_manager->host_data.pos_rigid[b2],
                               quaternion_conjugate);

                    rotated_point_b[i] = real3_int(sbar, b2);
                    quat_b[i] = quaternion_conjugate;
                }
            }
        }
    }

    void Project(real* gamma);
    void Project_Single(int index, real* gamma);
    void host_Project_single(int index, vec2* ids, real3* friction, real* cohesion, real* gamma);

    void func_Project_normal(int index, const vec2* ids, const real* cohesion, real* gam);
    void func_Project_sliding(int index, const vec2* ids, const real3* fric, const real* cohesion, real* gam);
    void func_Project_spinning(int index, const vec2* ids, const real3* fric, real* gam);
    void Dx(const DynamicVector<real>& x, DynamicVector<real>& output);
    void D_Tx(const DynamicVector<real>& x, DynamicVector<real>& output);

    // Compute the vector of corrections
    void Build_b();
    // Compute the diagonal compliance matrix
    void Build_E();
    // Compute the jacobian matrix, no allocation is performed here,
    // GenerateSparsity should take care of that
    void Build_D();
    void Build_s();
    // Fill-in the non zero entries in the bilateral jacobian with ones.
    // This operation is sequential.
    void GenerateSparsity();
    int offset;

  protected:
    custom_vector<bool2> contact_active_pairs;

    real inv_h;
    real inv_hpa;
    real inv_hhpa;
    custom_vector<real3_int> rotated_point_a, rotated_point_b;
    custom_vector<quaternion> quat_a, quat_b;
    // Pointer to the system's data manager
    ChParallelDataManager* data_manager;
};
}
