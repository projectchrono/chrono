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
    // Constructors
    Ch3DOFContainer();
    ~Ch3DOFContainer();

    Ch3DOFContainer(const Ch3DOFContainer& other);             // Copy constructor
    Ch3DOFContainer& operator=(const Ch3DOFContainer& other);  // Assignment operator

    // Before Solve
    virtual void Update(double ChTime){};
    virtual void Setup(int start_constraint);
    virtual void Initialize() {}
    virtual void ComputeInvMass(int offset) {}
    virtual void ComputeMass(int offset) {}
    virtual void GenerateSparsity() {}
    virtual void Build_D() {}
    virtual void Build_b() {}
    virtual void Build_E() {}
    virtual void PreSolve() {}
    virtual void ComputeDOF() {}

    // During Solve
    virtual void Project(real* gamma) {}
    virtual void UpdateRhs() {}
    // After Solve
    virtual void UpdatePosition(double ChTime) {}
    virtual void PostSolve() {}

    // Helper Functions
    virtual int GetNumConstraints() { return 0; }
    virtual int GetNumNonZeros() { return 0; }
    virtual void CalculateContactForces() {}
    virtual real3 GetBodyContactForce(uint body_id) {}
    virtual real3 GetBodyContactTorque(uint body_id) {}
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

    real kernel_radius;
    real collision_envelope;
    real contact_recovery_speed;  // The speed at which 'rigid' fluid  bodies resolve contact
    real contact_cohesion;
    real contact_mu;    // friction
    real max_velocity;  // limit on the maximum speed the fluid can move at
    uint start_row;

    uint start_boundary;
    uint start_density;
    uint start_viscous;

    // Store boundary forces here for rigid bodies
    DynamicVector<real> contact_forces;

  protected:
    ChParallelDataManager* data_manager;

    uint num_fluid_contacts;
    uint num_fluid_bodies;
    uint num_rigid_bodies;
    uint num_rigid_fluid_contacts;
    uint num_unilaterals;
    uint num_bilaterals;
    uint num_shafts;
    uint num_fea_tets;
    uint num_fea_nodes;
    uint num_mpm_markers;
    uint num_mpm_nodes;
};

class CH_PARALLEL_API ChFluidContainer : public Ch3DOFContainer {
  public:
    ChFluidContainer(ChSystemParallelDVI* system);
    ~ChFluidContainer();
    void AddFluid(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    void Update(double ChTime);
    void UpdatePosition(double ChTime);
    int GetNumConstraints();
    int GetNumNonZeros();
    void Setup(int start_constraint);
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
    void CalculateContactForces();
    real3 GetBodyContactForce(uint body_id);
    real3 GetBodyContactTorque(uint body_id);
    custom_vector<Mat33> shear_tensor;
    custom_vector<real> shear_trace;
    custom_vector<real> density;

    real compliance;
    real epsilon;  // Regularization parameter
    real tau;      // Constraint relaxation time
    real rho;
    real mass;
    real viscosity;
    bool artificial_pressure;  // Enable artificial pressure term
    real artificial_pressure_k;
    real artificial_pressure_n;
    real artificial_pressure_dq;
    bool enable_viscosity;
    bool initialize_mass;

  private:
    uint body_offset;
};

class CH_PARALLEL_API ChMPMContainer : public Ch3DOFContainer {
  public:
    ChMPMContainer(ChSystemParallelDVI* system);
    ~ChMPMContainer();
    void AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    void ComputeDOF();
    void Update(double ChTime);
    void UpdatePosition(double ChTime);
    void Setup(int start_constraint);
    void Initialize();
    void PreSolve();
    void Build_D();
    void Build_b();
    void Build_E();
    void UpdateRhs();
    void Solve(const DynamicVector<real>& s, SubVectorType& gamma);
    void Multiply(DynamicVector<real>& v_array, DynamicVector<real>& result_array);
    void Project(real* gamma) {}
    void GenerateSparsity();
    void ComputeInvMass(int offset);
    void ComputeMass(int offset);
    void PostSolve();
    int GetNumConstraints();
    int GetNumNonZeros();

    DynamicVector<real> grid_mass;
    DynamicVector<real> grid_vel_old;
    DynamicVector<real> volume;
    DynamicVector<real> rhs;
    custom_vector<Mat33> Fe, Fe_hat, Fp, delta_F;
    custom_vector<Mat33> Fe_node, Fp_node;

    DynamicVector<real> temp, ml, mg, mg_p, ml_candidate, ms, my, mdir, ml_p;
    DynamicVector<real> mD, invmD;

    uint start_node;
    uint num_mpm_constraints;
    real mass;
    real mu;
    real hardening_coefficient;
    real lambda;
    real theta_s;
    real theta_c;
    real alpha;
    real max_iterations;

    real3 min_bounding_point;
    real3 max_bounding_point;
    int3 bins_per_axis;
    real bin_edge;
    real inv_bin_edge;
    uint body_offset;
};
class CH_PARALLEL_API ChFEAContainer : public Ch3DOFContainer {
  public:
    ChFEAContainer(ChSystemParallelDVI* system);
    ~ChFEAContainer();
    void AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    void AddElements(const std::vector<uint4>& indices);
    // Compute initial shape matrix
    void Initialize();
    void Setup(int start_constraint);
    void Update(double ChTime);
    void UpdatePosition(double ChTime);
    void GenerateSparsity();
    int GetNumConstraints();
    int GetNumNonZeros();
    void Project(real* gamma);
    void Build_D();
    void Build_b();
    void Build_E();
    void ComputeInvMass(int offset);
    void ComputeMass(int offset);
    custom_vector<Mat33> X0;  // Inverse of intial shape matrix

    custom_vector<real> V;  // volume of tet
    real youngs_modulus;
    real poisson_ratio;
    real material_density;
    uint num_tet_constraints;  // Strain constraints + volume constraint
    uint start_tet;
    uint start_boundary;
};
}
