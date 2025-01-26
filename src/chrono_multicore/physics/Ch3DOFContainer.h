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
// Definitions for all 3DOF type containers for Chrono::Multicore.
//
// =============================================================================

#pragma once

#include <thread>

#include "chrono_multicore/ChConfigMulticore.h"
#include "chrono_multicore/ChMulticoreDefines.h"

#include "chrono/multicore_math/ChMulticoreMath.h"
#include "chrono/multicore_math/matrix.h"

// Chrono headers
#include "chrono/physics/ChBody.h"

// Blaze headers
// ATTENTION: It is important for these to be included after sse.h!
#include <blaze/math/DynamicVector.h>

using blaze::DynamicVector;

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystemMulticoreNSC;
class ChMulticoreDataManager;
class ChSolverMulticore;

/// @addtogroup multicore_physics
/// @{

/// Base class for containers of elements with 3 degrees of freedom (fluid nodes, rigid particles).
class CH_MULTICORE_API Ch3DOFContainer : public ChPhysicsItem {
  public:
    Ch3DOFContainer();
    virtual ~Ch3DOFContainer() {}

    Ch3DOFContainer(const Ch3DOFContainer& other);             // Copy constructor
    Ch3DOFContainer& operator=(const Ch3DOFContainer& other);  // Assignment operator

    // Before Solve
    virtual void Update3DOF(double ChTime) {}
    virtual void Setup3DOF(int start_constraint);
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
    virtual unsigned int GetNumConstraints() { return 0; }
    virtual unsigned int GetNumNonZeros() { return 0; }
    virtual void CalculateContactForces() {}
    virtual real3 GetBodyContactForce(std::shared_ptr<ChBody> body) { return real3(0); }
    virtual real3 GetBodyContactTorque(std::shared_ptr<ChBody> body) { return real3(0); }
    // Integrate happens after the solve
    // void Integrate(double ChTime);
    // Position of the node - in absolute csys.
    real3 GetPos(int i);
    // Position of the node - in absolute csys.
    void SetPos(const int& i, const real3& mpos);

    // Velocity of the node - in absolute csys.
    real3 GetPosDt(int i);
    // Velocity of the node - in absolute csys.
    void SetPosDt(const int& i, const real3& mposdt);

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
    ChMulticoreDataManager* data_manager;

    uint num_fluid_contacts;
    uint num_fluid_bodies;
    uint num_rigid_bodies;
    uint num_rigid_fluid_contacts;
    uint num_unilaterals;
    uint num_bilaterals;
    uint num_shafts;
    uint num_motors;

    friend class ChMulticoreDataManager;
    friend class ChSystemMulticoreNSC;
};

/// Container of fluid particles.
class CH_MULTICORE_API ChFluidContainer : public Ch3DOFContainer {
  public:
    ChFluidContainer();
    ~ChFluidContainer() {}

    void AddBodies(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    virtual void Update3DOF(double ChTime) override;
    virtual void UpdatePosition(double ChTime) override;
    virtual unsigned int GetNumConstraints() override;
    virtual unsigned int GetNumNonZeros() override;
    virtual void Setup3DOF(int start_constraint) override;
    virtual void Initialize() override;
    virtual void PreSolve() override;
    void Density_Fluid();
    void Normalize_Density_Fluid();
    virtual void Build_D() override;
    virtual void Build_b() override;
    virtual void Build_E() override;
    virtual void Project(real* gamma) override;
    virtual void GenerateSparsity() override;
    virtual void ComputeInvMass(int offset) override;
    virtual void ComputeMass(int offset) override;
    virtual void PostSolve() override;
    virtual void CalculateContactForces() override;
    virtual real3 GetBodyContactForce(std::shared_ptr<ChBody> body) override;
    virtual real3 GetBodyContactTorque(std::shared_ptr<ChBody> body) override;
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

  private:
    uint body_offset;
};

/// Container of rigid particles (3 DOF).
class CH_MULTICORE_API ChParticleContainer : public Ch3DOFContainer {
  public:
    ChParticleContainer();
    ~ChParticleContainer() {}

    void AddBodies(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    virtual void Update3DOF(double ChTime) override;
    virtual void UpdatePosition(double ChTime) override;
    virtual unsigned int GetNumConstraints() override;
    virtual unsigned int GetNumNonZeros() override;
    virtual void Setup3DOF(int start_constraint) override;
    virtual void Initialize() override;
    virtual void PreSolve() override;
    virtual void Build_D() override;
    virtual void Build_b() override;
    virtual void Build_E() override;
    virtual void Project(real* gamma) override;
    virtual void GenerateSparsity() override;
    virtual void ComputeInvMass(int offset) override;
    virtual void ComputeMass(int offset) override;
    virtual void PostSolve() override;
    virtual void CalculateContactForces() override;
    virtual real3 GetBodyContactForce(std::shared_ptr<ChBody> body) override;
    virtual real3 GetBodyContactTorque(std::shared_ptr<ChBody> body) override;
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

  private:
    uint body_offset;
};

/// @} multicore_physics

}  // end namespace chrono
