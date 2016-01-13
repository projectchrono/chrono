// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Description: This file defines a fluid node, it is based on a 3DOF NodeXYZ
// element. This class is similar to ChMatterSPH but is meant to be a bit more
// general.
// =============================================================================

#pragma once

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real.h"
#include "chrono/physics/ChPhysicsItem.h"
#include <blaze/math/DynamicVector.h>

using blaze::DynamicVector;

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystemParallelDVI;
class ChSystemParallelMPM;
class ChParallelDataManager;

class CH_PARALLEL_API Ch3DOFContainer : public ChPhysicsItem {
  public:
    Ch3DOFContainer();
    ~Ch3DOFContainer();

    Ch3DOFContainer(const Ch3DOFContainer& other);             // Copy constructor
    Ch3DOFContainer& operator=(const Ch3DOFContainer& other);  // Assignment operator

    // void AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    // Update occurs before he solve
    virtual void Update(double ChTime) = 0;
    virtual void UpdatePosition(double ChTime) = 0;
    virtual int GetNumConstraints() = 0;
    virtual void Setup() = 0;
    virtual void Initialize() = 0;
    virtual void PreSolve() = 0;
    virtual void Build_D() = 0;
    virtual void Build_b() = 0;
    virtual void Build_E() = 0;
    virtual void Project(real* gamma) = 0;
    virtual void GenerateSparsity() = 0;
    virtual void ComputeInvMass(int offset) = 0;
    virtual void ComputeMass(int offset) = 0;
    virtual void PostSolve() = 0;
    // Integrate happens after the solve
    // void Integrate(double ChTime);
    // Position of the node - in absolute csys.
    real3 GetPos(int i);
    // Position of the node - in absolute csys.
    void SetPos(const int& i, const real3& mpos);

    // Velocity of the node - in absolute csys.
    real3 GetPos_dt(int i);
    // Velocity of the node - in absolute csys.
    void SetPos_dt(const int& i, const real3& mposdt);

  protected:
    ChParallelDataManager* data_manager;
};

class CH_PARALLEL_API ChFluidContainer : public Ch3DOFContainer {
  public:
    ChFluidContainer(ChSystemParallelDVI* system);
    ~ChFluidContainer();
    void AddFluid(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    void Update(double ChTime);
    void UpdatePosition(double ChTime);
    int GetNumConstraints();
    void Setup();
    void Initialize();
    void PreSolve() {}
    void Density_Fluid();
    void Normalize_Density_Fluid();
    void Build_D();
    void Build_b();
    void Build_E();
    void Project(real* gamma);
    void GenerateSparsity();
    void ComputeInvMass(int offset);
    void ComputeMass(int offset);
    void PostSolve();

    custom_vector<Mat33> shear_tensor;
    custom_vector<real> shear_trace;
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

    custom_vector<real3> den_con_jac;
    custom_vector<real3> visc1_jac;
    custom_vector<real3> visc2_jac;
    custom_vector<real3> visc3_jac;
};
class CH_PARALLEL_API ChMPMContainer : public Ch3DOFContainer {
  public:
    ChMPMContainer(ChSystemParallelDVI* system);
    ~ChMPMContainer();
    void AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    void Update(double ChTime);
    void UpdatePosition(double ChTime);
    void Setup() {}
    void Initialize();
    void Multiply(DynamicVector<real>& v_array, DynamicVector<real>& result_array);
    void Solve(const DynamicVector<real>& b, DynamicVector<real>& x);
    void PreSolve();
    void Build_D() {}
    void Build_b() {}
    void Build_E() {}
    void Project(real* gamma) {}
    void GenerateSparsity() {}
    void ComputeInvMass(int offset);
    void ComputeMass(int offset);
    void PostSolve() {}
    int GetNumConstraints() { return 0; }

    DynamicVector<real> grid_mass;
    DynamicVector<real> grid_vel;
    DynamicVector<real> grid_vel_old;
    custom_vector<real3> grid_forces;
    DynamicVector<real> volume, rhs;
    custom_vector<Mat33> Fe, Fe_hat, Fp, delta_F;

    DynamicVector<real> r, p, Ap, q, s;
};
// class CH_PARALLEL_API ChFEMContainer : public Ch3DOFContainer {
//  public:
//    ChFEMContainer(ChSystemParallelDVI* system);
//    ~ChFEMContainer();
//};
}
