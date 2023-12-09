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
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: The definition of a multicore ChSystem, pretty much everything
// is done manually instead of using the functions used in ChSystem. This is to
// handle the different data structures present in the multicore implementation
//
// =============================================================================

#pragma once

#include <cstdlib>
#include <cfloat>
#include <memory>
#include <algorithm>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono/multicore_math/ChMulticoreMath.h"

#include "chrono_multicore/physics/Ch3DOFContainer.h"
#include "chrono_multicore/ChDataManager.h"
#include "chrono_multicore/ChMulticoreDefines.h"
#include "chrono_multicore/ChSettings.h"
#include "chrono_multicore/ChMeasures.h"

namespace chrono {

class ChMulticoreDataManager;
class settings_container;

/// @addtogroup multicore_physics
/// @{

/// Base class for Chrono::Multicore systems.
class CH_MULTICORE_API ChSystemMulticore : public ChSystem {

  public:
    ChSystemMulticore();
    ChSystemMulticore(const ChSystemMulticore& other);
    virtual ~ChSystemMulticore();

    virtual bool Integrate_Y() override;
    virtual void AddBody(std::shared_ptr<ChBody> newbody) override;
    virtual void AddShaft(std::shared_ptr<ChShaft> shaft) override;
    virtual void AddLink(std::shared_ptr<ChLinkBase> link) override;
    virtual void AddOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> newitem) override;

    void ClearForceVariables();
    virtual void Update();
    virtual void UpdateBilaterals();
    virtual void UpdateLinks();
    virtual void UpdateOtherPhysics();
    virtual void UpdateRigidBodies();
    virtual void UpdateShafts();
    virtual void UpdateMotorLinks();
    virtual void Update3DOFBodies();
    void RecomputeThreads();

    virtual void AddMaterialSurfaceData(std::shared_ptr<ChBody> newbody) = 0;
    virtual void UpdateMaterialSurfaceData(int index, ChBody* body) = 0;
    virtual void Setup() override;
    virtual void SetCollisionSystemType(ChCollisionSystem::Type type) override;

    /// Change the default composition laws for contact surface materials
    /// (coefficient of friction, cohesion, compliance, etc.).
    virtual void SetMaterialCompositionStrategy(std::unique_ptr<ChMaterialCompositionStrategy>&& strategy) override;

    virtual void PrintStepStats();
    unsigned int GetNumBodies();
    unsigned int GetNumShafts();
    unsigned int GetNumContacts();
    unsigned int GetNumBilaterals();

    /// Return the time (in seconds) spent for computing the time step.
    virtual double GetTimerStep() const override;

    /// Return the time (in seconds) for time integration, within the time step.
    virtual double GetTimerAdvance() const override;

    /// Return the time (in seconds) for the solver, within the time step.
    /// Note that this time excludes any calls to the solver's Setup function.
    virtual double GetTimerLSsolve() const override;

    /// Return the time (in seconds) for the solver Setup phase, within the time step.
    virtual double GetTimerLSsetup() const override;

    /// Return the time (in seconds) for calculating/loading Jacobian information, within the time step.
    virtual double GetTimerJacobian() const override;

    /// Return the time (in seconds) for runnning the collision detection step, within the time step.
    virtual double GetTimerCollision() const override;

    /// Return the time (in seconds) for system setup, within the time step.
    virtual double GetTimerSetup() const override { return 0; }

    /// Return the time (in seconds) for updating auxiliary data, within the time step.
    virtual double GetTimerUpdate() const override;

    /// Calculate cummulative contact forces for all bodies in the system.
    /// Note that this function must be explicitly called by the user at each time where
    /// calls to GetContactableForce or ContactableTorque are made.
    virtual void CalculateContactForces() {}

    /// Return the resultant applied force on the specified body.
    /// This resultant force includes all external applied loads acting on the body (from gravity, loads, springs,
    /// etc). However, this does *not* include any constraint forces. In particular, contact forces are not included if
    /// using the NSC formulation, but are included when using the SMC formulation.
    virtual ChVector<> GetBodyAppliedForce(ChBody* body) override;

    /// Return the resultant applied torque on the specified body.
    /// This resultant torque includes all external applied loads acting on the body (from gravity, loads, springs,
    /// etc). However, this does *not* include any constraint forces. In particular, contact torques are not included if
    /// using the NSC formulation, but are included when using the SMC formulation.
    virtual ChVector<> GetBodyAppliedTorque(ChBody* body) override;

    /// Get the contact force on the body with specified id.
    /// Note that ComputeContactForces must be called prior to calling this function
    /// at any time where reporting of contact forces is desired.
    virtual real3 GetBodyContactForce(uint body_id) const = 0;

    /// Get the contact torque on the body with specified id.
    /// Note that ComputeContactForces must be called prior to calling this function
    /// at any time where reporting of contact torques is desired.
    virtual real3 GetBodyContactTorque(uint body_id) const = 0;

    /// Get the contact force on the specified body.
    /// Note that ComputeContactForces must be called prior to calling this function
    /// at any time where reporting of contact forces is desired.
    real3 GetBodyContactForce(std::shared_ptr<ChBody> body) const { return GetBodyContactForce(body->GetId()); }

    /// Get the contact torque on the specified body.
    /// Note that ComputeContactForces must be called prior to calling this function
    /// at any time where reporting of contact torques is desired.
    real3 GetBodyContactTorque(std::shared_ptr<ChBody> body) const { return GetBodyContactTorque(body->GetId()); }

    settings_container* GetSettings();

    /// Set the number of OpenMP threads used by Chrono itself, Eigen, and the collision detection system.
    /// <pre>
    ///  num_threads_chrono    - used for all OpenMP constructs and thrust algorithms in Chrono::Multicore.
    ///  num_threads_collision - Ignored.  Chrono::Multicore sets num_threads_collision = num_threads_chrono.
    ///  num_threads_eigen     - used in the Eigen sparse direct solvers and a few linear algebra operations.
    ///                          Note that Eigen enables multi-threaded execution only under certain size conditions.
    ///                          See the Eigen documentation.
    ///                          If passing 0, then num_threads_eigen = num_threads_chrono.
    /// </pre>
    /// By default, num_threads_chrono is set to omp_get_num_procs() and num_threads_eigen is set to 1.
    virtual void SetNumThreads(int num_threads_chrono,
                               int num_threads_collision = 0,
                               int num_threads_eigen = 0) override;

    /// Enable dynamic adjustment of number of threads between the specified limits.
    /// The initial number of threads is set to min_threads.
    void EnableThreadTuning(int min_threads, int max_threads);

    /// Calculate the (linearized) bilateral constraint violations.
    /// Return the maximum constraint violation.
    double CalculateConstraintViolation(std::vector<double>& cvec);

    ChMulticoreDataManager* data_manager;

    int current_threads;

  protected:
    double old_timer, old_timer_cd;
    bool detect_optimal_threads;

    int detect_optimal_bins;
    std::vector<double> timer_accumulator, cd_accumulator;
    uint frame_threads, frame_bins, counter;
    std::vector<ChLink*>::iterator it;

  private:
    std::vector<ChLinkMotorLinearSpeed*> linmotorlist;
    std::vector<ChLinkMotorRotationSpeed*> rotmotorlist;
};

//====================================================================================================

/// Multicore system using non-smooth contact (complementarity-based) method.
class CH_MULTICORE_API ChSystemMulticoreNSC : public ChSystemMulticore {

  public:
    ChSystemMulticoreNSC();
    ChSystemMulticoreNSC(const ChSystemMulticoreNSC& other);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChSystemMulticoreNSC* Clone() const override { return new ChSystemMulticoreNSC(*this); }

    void ChangeSolverType(SolverType type);
    void Initialize();

    virtual ChContactMethod GetContactMethod() const override { return ChContactMethod::NSC; }
    virtual void AddMaterialSurfaceData(std::shared_ptr<ChBody> newbody) override;
    virtual void UpdateMaterialSurfaceData(int index, ChBody* body) override;

    void Add3DOFContainer(std::shared_ptr<Ch3DOFContainer> container);

    virtual void SetContactContainer(std::shared_ptr<ChContactContainer> container) override;

    void CalculateContactForces() override;
    real CalculateKineticEnergy();
    real CalculateDualObjective();

    virtual real3 GetBodyContactForce(uint body_id) const override;
    virtual real3 GetBodyContactTorque(uint body_id) const override;
    using ChSystemMulticore::GetBodyContactForce;
    using ChSystemMulticore::GetBodyContactTorque;

    virtual void AssembleSystem();
    virtual void SolveSystem();
};

//====================================================================================================

/// Multicore system using smooth contact (penalty-based) method.
class CH_MULTICORE_API ChSystemMulticoreSMC : public ChSystemMulticore {

  public:
    ChSystemMulticoreSMC();
    ChSystemMulticoreSMC(const ChSystemMulticoreSMC& other);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChSystemMulticoreSMC* Clone() const override { return new ChSystemMulticoreSMC(*this); }

    virtual ChContactMethod GetContactMethod() const override { return ChContactMethod::SMC; }
    virtual void AddMaterialSurfaceData(std::shared_ptr<ChBody> newbody) override;
    virtual void UpdateMaterialSurfaceData(int index, ChBody* body) override;

    virtual void Setup() override;
    virtual void SetCollisionSystemType(ChCollisionSystem::Type type) override;
    virtual void SetContactContainer(std::shared_ptr<ChContactContainer> container) override;

    virtual real3 GetBodyContactForce(uint body_id) const override;
    virtual real3 GetBodyContactTorque(uint body_id) const override;
    using ChSystemMulticore::GetBodyContactForce;
    using ChSystemMulticore::GetBodyContactTorque;

    virtual void PrintStepStats() override;

    double GetTimerProcessContact() const {
        return data_manager->system_timer.GetTime("ChIterativeSolverMulticoreSMC_ProcessContact");
    }
};

/// @} multicore_physics

}  // end namespace chrono
