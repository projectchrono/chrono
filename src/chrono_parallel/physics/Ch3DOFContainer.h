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
// Definitions for all 3DOF type containers for chrono parallel.
//
// =============================================================================

#pragma once

#include <thread>

// Chrono::Parallel headers
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/other_types.h"
#include "chrono_parallel/math/matrix.h"
#include "chrono_parallel/math/sse.h"
#include "chrono_parallel/physics/ChMPMSettings.h"

// Chrono headers
#include "chrono/physics/ChBody.h"

// Blaze headers
// ATTENTION: It is important for these to be included after sse.h!
#include <blaze/math/DynamicVector.h>

using blaze::DynamicVector;

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystemParallelNSC;
class ChSystemParallelMPM;
class ChParallelDataManager;
class ChSolverParallel;

/// @addtogroup parallel_physics
/// @{

/// Base class for containers of elements with 3 degrees of freedom
/// (fluid nodes, particles, FEA nodes).
class CH_PARALLEL_API Ch3DOFContainer : public ChPhysicsItem {
  public:
    Ch3DOFContainer();
    virtual ~Ch3DOFContainer() {}

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
    // Does one iteration of a solve
    virtual void InnerSolve() {}

    // During Solve
    virtual void Project(real* gamma) {}
    virtual void UpdateRhs() {}
    // After Solve
    virtual void UpdatePosition(double ChTime) {}
    virtual void PostSolve() {}
    void SetFamily(short family, short mask_no_collision);

    // Helper Functions
    uint GetNumParticles() const { return num_fluid_bodies; }
    virtual int GetNumConstraints() { return 0; }
    virtual int GetNumNonZeros() { return 0; }
    virtual void CalculateContactForces() {}
    virtual real3 GetBodyContactForce(uint body_id) { return real3(0); }
    virtual real3 GetBodyContactTorque(uint body_id) { return real3(0); }
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
    real contact_compliance;
    real contact_mu;    // friction
    real max_velocity;  // limit on the maximum speed the fluid can move at
    uint start_row;
    real alpha;

    // Store boundary forces here for rigid bodies
    DynamicVector<real> contact_forces;
    DynamicVector<real> gamma_old;
    short2 family;

  protected:
    ChParallelDataManager* data_manager;

    uint num_fluid_contacts;
    uint num_fluid_bodies;
    uint num_rigid_bodies;
    uint num_rigid_fluid_contacts;
    uint num_rigid_mpm_contacts;
    uint num_unilaterals;
    uint num_bilaterals;
    uint num_shafts;
    uint num_motors;
    uint num_fea_tets;
    uint num_fea_nodes;

    friend class ChParallelDataManager;
    friend class ChSystemParallelNSC;
};

/// Container of fluid particles.
class CH_PARALLEL_API ChFluidContainer : public Ch3DOFContainer {
  public:
    ChFluidContainer();
    ~ChFluidContainer() {}

    void AddBodies(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    void Update(double ChTime);
    void UpdatePosition(double ChTime);
    int GetNumConstraints();
    int GetNumNonZeros();
    void Setup(int start_constraint);
    void Initialize();
    void PreSolve();
    void Density_Fluid();
    void Density_FluidMPM();
    void DensityConstraint_FluidMPM();
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
    void GetFluidDensity(custom_vector<real>& dens);
    void GetFluidPressure(custom_vector<real>& pres);
    void GetFluidForce(custom_vector<real3>& forc);
    custom_vector<Mat33> shear_tensor;
    custom_vector<real> shear_trace;
    custom_vector<real> density;

    uint start_boundary;
    uint start_density;
    uint start_viscous;
    real yield_stress;
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

    real nu;
    real youngs_modulus;
    real hardening_coefficient;
    real lame_lambda;
    real lame_mu;
    real theta_s;
    real theta_c;
    real alpha_flip;

    int mpm_iterations;
    std::thread mpm_thread;
    bool mpm_init;
    MPM_Settings temp_settings;
    custom_vector<float> mpm_pos, mpm_vel, mpm_jejp;

  private:
    uint body_offset;
};

/// Container of FEA nodes.
class CH_PARALLEL_API ChFEAContainer : public Ch3DOFContainer {
  public:
    ChFEAContainer();
    ~ChFEAContainer() {}

    void AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    void AddElements(const std::vector<uvec4>& indices);
    void AddConstraint(const uint node, std::shared_ptr<ChBody>& body);
    // Compute initial shape matrix
    void Initialize();
    void Setup(int start_constraint);
    void Update(double ChTime);
    void UpdatePosition(double ChTime);
    void GenerateSparsity();
    int GetNumConstraints();
    int GetNumNonZeros();
    void Project(real* gamma);
    void FindSurface();
    void Build_D();
    void Build_b();
    void Build_E();

    void PreSolve();
    void PostSolve();

    void ComputeInvMass(int offset);
    void ComputeMass(int offset);
    custom_vector<Mat33> X0;  // Inverse of intial shape matrix

    int num_boundary_triangles;
    int num_boundary_elements;
    int num_boundary_nodes;

    custom_vector<real> V;  // volume of tet

    real youngs_modulus;
    real poisson_ratio;
    real material_density;
    real beta;
    uint num_tet_constraints;  // Strain constraints + volume constraint
    uint start_tet;
    uint start_boundary;
    uint start_boundary_node;
    uint start_boundary_marker;
    uint start_rigid;

    // Id of the rigid body and node number
    custom_vector<int> constraint_bodies;
    std::vector<std::shared_ptr<ChBody>> bodylist;

    real rigid_constraint_recovery_speed;
    // The point where the constraint is enforced in the local coords of the rigid body
    custom_vector<real3> constraint_position;
    custom_vector<quaternion> constraint_rotation;
    DynamicVector<real> gamma_old_rigid;

    uint num_rigid_constraints;
};

/// Container of rigid particles (3 DOF).
class CH_PARALLEL_API ChParticleContainer : public Ch3DOFContainer {
  public:
    ChParticleContainer();
    ~ChParticleContainer() {}

    void AddBodies(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    void Update(double ChTime);
    void UpdatePosition(double ChTime);
    int GetNumConstraints();
    int GetNumNonZeros();
    void Setup(int start_constraint);
    void Initialize();
    void PreSolve();
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
    void GetFluidForce(custom_vector<real3>& forc);

    uint start_boundary;
    uint start_contact;
    real compliance;
    real mu;
    real cohesion;
    real mass;
    uint num_rigid_contacts;  // number of rigid contacts without duplicates or self contacts
    real yield_stress;

    real nu;
    real youngs_modulus;
    real hardening_coefficient;
    real lame_lambda;
    real lame_mu;
    real theta_s;
    real theta_c;
    real alpha_flip;

    int mpm_iterations;
    custom_vector<float> mpm_pos, mpm_vel, mpm_jejp;

    std::thread mpm_thread;
    bool mpm_init;
    MPM_Settings temp_settings;

  private:
    uint body_offset;
};

/// @} parallel_physics

} // end namespace chrono
