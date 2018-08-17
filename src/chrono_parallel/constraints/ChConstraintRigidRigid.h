// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Handling of rigid contact and calculation of corrections and Jacobians
//
// =============================================================================

#pragma once

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"

namespace chrono {

/// @addtogroup parallel_constraint
/// @{

/// Unilateral (contact) constraints.
class CH_PARALLEL_API ChConstraintRigidRigid {
  public:
    ChConstraintRigidRigid();
    ~ChConstraintRigidRigid() {}

    void Setup(ChParallelDataManager* dm);
    void Project(real* gamma);
    void Project_Single(int index, real* gamma);
    void host_Project_single(int index, vec2* ids, real3* friction, real* cohesion, real* gamma);

    void func_Project_normal(int index, const vec2* ids, const real* cohesion, real* gam);
    void func_Project_sliding(int index, const vec2* ids, const real3* fric, const real* cohesion, real* gam);
    void func_Project_spinning(int index, const vec2* ids, const real3* fric, real* gam);
    void Dx(const DynamicVector<real>& x, DynamicVector<real>& output);
    void D_Tx(const DynamicVector<real>& x, DynamicVector<real>& output);

    /// Compute the vector of corrections.
    void Build_b();
    /// Compute the diagonal compliance matrix.
    void Build_E();
    /// Compute the jacobian matrix, no allocation is performed here,
    /// GenerateSparsity should take care of that.
    void Build_D();
    void Build_s();
    /// Fill-in the non zero entries in the bilateral jacobian with ones.
    /// This operation is sequential.
    void GenerateSparsity();

    int offset;

  protected:
    custom_vector<bool2> contact_active_pairs;

    real inv_h;     ///< reciprocal of time step, 1/h
    real inv_hpa;   ///< 1 / (h+a)
    real inv_hhpa;  ///< 1 / h*(h+a)

    custom_vector<real3_int> rotated_point_a, rotated_point_b;
    custom_vector<quaternion> quat_a, quat_b;

    ChParallelDataManager* data_manager;  ///< Pointer to the system's data manager
};

/// @} parallel_constraint

} // end namespace chrono
