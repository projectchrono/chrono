// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHSYSTEM_H
#define CHSYSTEM_H

#include <cfloat>
#include <memory>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <iostream>
#include <list>

#include "chrono/collision/ChCCollisionSystem.h"
#include "chrono/core/ChLog.h"
#include "chrono/core/ChMath.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChAssembly.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChControls.h"
#include "chrono/physics/ChGlobal.h"
#include "chrono/physics/ChLinksAll.h"
#include "chrono/physics/ChProbe.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/timestepper/ChAssemblyAnalysis.h"
#include "chrono/solver/ChSolver.h"
#include "chrono/timestepper/ChIntegrable.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/timestepper/ChTimestepperHHT.h"

namespace chrono {

/// Physical system.
///
/// This class is used to represent a multibody physical system,
/// so it acts also as a database for most items involved in
/// simulations, most noticeably objects of ChBody and ChLink
/// classes, which are used to represent mechanisms.
///
/// Moreover, it also owns some global settings and features,
/// like the gravity acceleration, the global time and so on.
///
/// This object will be responsible of performing the entire
/// physical simulation (dynamics, kinematics, statics, etc.),
/// so you need at least one ChSystem object in your program, in
/// order to perform simulations (you'll insert rigid bodies and
/// links into it..)
///
/// Note that this is an abstract class, in your code you must
/// create a system from one of the concrete classes: 
///   @ref chrono::ChSystemNSC (for non-smooth contacts) or
///   @ref chrono::ChSystemSMC (for smooth 'penalty' contacts).
///
/// Further info at the @ref simulation_system  manual page.

class ChApi ChSystem : public ChAssembly, public ChIntegrableIIorder {

  public:
    /// Create a physical system.
    ChSystem();

    /// Copy constructor
    ChSystem(const ChSystem& other);

    /// Destructor
    virtual ~ChSystem();

    //
    // PROPERTIES
    //

    /// Sets the time step used for integration (dynamical simulation).
    /// The lower this value, the more precise the simulation. Usually, values
    /// about 0.01 s are enough for simple simulations. It may be modified automatically
    /// by integration methods, if they support automatic time adaption.
    void SetStep(double m_step) {
        if (m_step > 0)
            step = m_step;
    }
    /// Gets the current time step used for the integration (dynamical simulation).
    double GetStep() const { return step; }

    /// Sets the end of simulation.
    void SetEndTime(double m_end_time) { end_time = m_end_time; }
    /// Gets the end of the simulation
    double GetEndTime() const { return end_time; }

    /// Sets the lower limit for time step (only needed if using
    /// integration methods which support time step adaption).
    void SetStepMin(double m_step_min) {
        if (m_step_min > 0.)
            step_min = m_step_min;
    }
    /// Gets the lower limit for time step
    double GetStepMin() const { return step_min; }

    /// Sets the upper limit for time step (only needed if using
    /// integration methods which support time step adaption).
    void SetStepMax(double m_step_max) {
        if (m_step_max > step_min)
            step_max = m_step_max;
    }
    /// Gets the upper limit for time step
    double GetStepMax() const { return step_max; }

    /// Set the method for time integration (time stepper type).
    ///   - Suggested for fast dynamics with hard (NSC) contacts: EULER_IMPLICIT_LINEARIZED
    ///   - Suggested for fast dynamics with hard (NSC) contacts and low inter-penetration: EULER_IMPLICIT_PROJECTED
    ///   - Suggested for finite element smooth dynamics: HHT, EULER_IMPLICIT_LINEARIZED
    ///
    /// *Notes*:
    ///   - for more advanced customization, use SetTimestepper()
    ///   - old methods ANITESCU and TASORA were replaced by EULER_IMPLICIT_LINEARIZED and EULER_IMPLICIT_PROJECTED,
    ///     respectively
    void SetTimestepperType(ChTimestepper::Type type);

    /// Get the current method for time integration (time stepper type).
    ChTimestepper::Type GetTimestepperType() const { return timestepper->GetType(); }

    /// Set the timestepper object to be used for time integration.
    void SetTimestepper(std::shared_ptr<ChTimestepper> mstepper) { timestepper = mstepper; }

    /// Get the timestepper currently used for time integration
    std::shared_ptr<ChTimestepper> GetTimestepper() const { return timestepper; }

    /// Sets outer iteration limit for assembly constraints. When trying to keep constraints together,
    /// the iterative process is stopped if this max.number of iterations (or tolerance) is reached.
    void SetMaxiter(int m_maxiter) { maxiter = m_maxiter; }
    /// Gets iteration limit for assembly constraints.
    int GetMaxiter() const { return maxiter; }

    /// Sets tolerance (in m) for assembly constraints. When trying to keep constraints together,
    /// the iterative process is stopped if this tolerance (or max.number of iterations ) is reached
    void SetTol(double tolerance) { tol = tolerance; }
    /// Gets current tolerance for assembly constraints.
    double GetTol() const { return tol; }

    /// Sets tolerance for satisfying constraints at the velocity level.
    /// The tolerance specified here is in fact a tolerance at the force level.
    /// this value is multiplied by the value of the current time step and then
    /// used as a stopping criteria for the iterative speed solver.
    void SetTolForce(double tolerance) { tol_force = tolerance; }
    /// Return the current value of the tolerance used in the speed solver.
    double GetTolForce() const { return tol_force; }

    /// Change the default composition laws for contact surface materials
    /// (coefficient of friction, cohesion, compliance, etc.)
    void SetMaterialCompositionStrategy(std::unique_ptr<ChMaterialCompositionStrategy<float>>&& strategy);

    /// Accessor for the current composition laws for contact surface material.
    const ChMaterialCompositionStrategy<float>& GetMaterialCompositionStrategy() const { return *composition_strategy; }

    /// For elastic collisions, with objects that have nonzero
    /// restitution coefficient: objects will rebounce only if their
    /// relative colliding speed is above this threshold. Default 0.15 m/s.
    /// If this is too low, aliasing problems can happen with small high frequency
    /// rebounces, and settling to static stacking might be more difficult.
    void SetMinBounceSpeed(double mval) { min_bounce_speed = mval; }
    /// Objects will rebounce only if their relative colliding speed is above this threshold.
    double GetMinBounceSpeed() const { return min_bounce_speed; }

    /// For the default stepper, you can limit the speed of exiting from penetration
    /// situations. Usually set a positive value, about 0.1 .. 2 . (as exiting speed, in m/s)
    void SetMaxPenetrationRecoverySpeed(double mval) { max_penetration_recovery_speed = mval; }
    /// Get the limit on the speed for exiting from penetration situations (for Anitescu stepper)
    double GetMaxPenetrationRecoverySpeed() const { return max_penetration_recovery_speed; }

    /// Choose the solver type, to be used for the simultaneous solution of the constraints
    /// in dynamical simulations (as well as in kinematics, statics, etc.)
    ///   - Suggested solver for speed, but lower precision: SOR
    ///   - Suggested solver for higher precision: BARZILAIBORWEIN or APGD
    ///   - For problems that involve a stiffness matrix: MINRES
    ///
    /// *Notes*:
    ///   - Do not use CUSTOM type, as this type is reserved for external solvers
    ///     (set using SetSolver() and/or SetStabSolver())
    ///   - This function is a shortcut, internally equivalent to two calls to
    ///     SetSolver() and SetStabSolve()
    virtual void SetSolverType(ChSolver::Type type);

    /// Gets the current solver type.
    ChSolver::Type GetSolverType() const { return solver_speed->GetType(); }

    /// When using an iterative solver (es. SOR) set the maximum number of iterations.
    /// The higher the iteration number, the more precise the simulation (but more CPU time).
    void SetMaxItersSolverSpeed(int mval) { max_iter_solver_speed = mval; }
    /// Current maximum number of iterations, if using an iterative solver.
    int GetMaxItersSolverSpeed() const { return max_iter_solver_speed; }

    /// When using an iterative solver (es. SOR) and a timestepping method
    /// requiring post-stabilization (e.g., EULER_IMPLICIT_PROJECTED), set the
    /// the maximum number of stabilization iterations. The higher the iteration
    /// number, the more precise the simulation (but more CPU time).
    void SetMaxItersSolverStab(int mval) { max_iter_solver_stab = mval; }
    /// Current maxi. number of iterations, if using an iterative solver for stabilization.
    int GetMaxItersSolverStab() const { return max_iter_solver_stab; }

    /// If you want to easily turn ON/OFF the warm starting feature of both iterative solvers
    /// (the one for speed and the other for pos.stabilization) you can simply use the
    /// following instead of accessing them directly with GetSolver() and GetStabSolver()
    void SetSolverWarmStarting(bool usewarm = true);
    /// Tell if the warm starting is enabled for the speed solver, (if iterative type).
    bool GetSolverWarmStarting() const;

    /// If you want to easily adjust the omega overrelaxation parameter of both iterative solvers
    /// (the one for speed and the other for position stabilization) you can simply use the
    /// following instead of accessing them directly with GetSolver() and GetStabSolver().
    /// Note, usually a good omega for Jacobi or GPU solver is 0.2; for other iter.solvers can be up to 1.0
    void SetSolverOverrelaxationParam(double momega = 1.0);
    /// Tell the omega overrelaxation factor for the speed solver, (if iterative type).
    double GetSolverOverrelaxationParam() const;

    /// If you want to easily adjust the 'sharpness lambda' parameter of both iterative solvers
    /// (the one for speed and the other for pos.stabilization) you can simply use the
    /// following instead of accessing them directly with GetSolver() and GetStabSolver().
    /// Note, usually a good sharpness value is in 1..0.8 range (the lower, the more it helps exact
    /// convergence, but overall convergence gets also much slower so maybe better to tolerate some error)
    void SetSolverSharpnessParam(double momega = 1.0);
    /// Tell the 'sharpness lambda' factor for the speed solver, (if iterative type).
    double GetSolverSharpnessParam() const;

    /// Instead of using SetSolverType(), you can create your own custom solver (inherited from ChSolver)
    /// and plug it into the system using this function. 
    virtual void SetStabSolver(std::shared_ptr<ChSolver> newsolver);

    /// Access directly the stabilization solver, configured to be used for the stabilization
    /// of constraints (solve delta positions).
    virtual std::shared_ptr<ChSolver> GetStabSolver();

    /// Instead of using SetSolverType(), you can create your own custom solver (suffice it is inherited
    /// from ChSolver) and plug it into the system using this function.
    virtual void SetSolver(std::shared_ptr<ChSolver> newsolver);

    /// Access directly the solver, configured to be used for the main differential
    /// inclusion problem (on speed-impulses).
    virtual std::shared_ptr<ChSolver> GetSolver();

    /// Instead of using the default 'system descriptor', you can create your own custom descriptor
    /// (inherited from ChSystemDescriptor) and plug it into the system using this function.
    void SetSystemDescriptor(std::shared_ptr<ChSystemDescriptor> newdescriptor);

    /// Access directly the 'system descriptor'.
    std::shared_ptr<ChSystemDescriptor> GetSystemDescriptor() { return descriptor; }

    /// Changes the number of parallel threads (by default is n.of cores).
    /// Note that not all solvers use parallel computation.
    /// If you have a N-core processor, this should be set at least =N for maximum performance.
    void SetParallelThreadNumber(int mthreads = 2);
    /// Get the number of parallel threads.
    /// Note that not all solvers use parallel computation.
    int GetParallelThreadNumber() { return parallel_thread_number; }

    /// Sets the G (gravity) acceleration vector, affecting all the bodies in the system.
    void Set_G_acc(const ChVector<>& m_acc) { G_acc = m_acc; }
    /// Gets the G (gravity) acceleration vector affecting all the bodies in the system.
    const ChVector<>& Get_G_acc() const { return G_acc; }

    /// Initial system setup before analysis.
    /// This function must be called once the system construction is completed.
    void SetupInitial() override;

    //
    // DATABASE HANDLING.
    //

    /// Removes all bodies/marker/forces/links/contacts,
    /// also resets timers and events.
    void Clear();

    /// Return the contact method supported by this system.
    /// Bodies added to this system must be compatible.
    virtual ChMaterialSurface::ContactMethod GetContactMethod() const = 0;

    /// Create and return the pointer to a new body.
    /// The returned body is created with a contact model consistent with the type
    /// of this ChSystem and with the collision system currently associated with this
    /// ChSystem.  Note that the body is *not* attached to this system.
    virtual ChBody* NewBody() = 0;

    /// Create and return the pointer to a new body with auxiliary reference frame.
    /// The returned body is created with a contact model consistent with the type
    /// of this ChSystem and with the collision system currently associated with this
    /// ChSystem.  Note that the body is *not* attached to this system.
    virtual ChBodyAuxRef* NewBodyAuxRef() = 0;

    /// Attach a probe to this system.
    void AddProbe(const std::shared_ptr<ChProbe>& newprobe);
    /// Attach a control to this system.
    void AddControls(const std::shared_ptr<ChControls>& newcontrols);

    /// Remove all probes from this system.
    void RemoveAllProbes();
    /// Remove all controls from this system.
    void RemoveAllControls();

    /// Replace the contact container.
    virtual void SetContactContainer(std::shared_ptr<ChContactContainer> container);

    /// Get the contact container
    std::shared_ptr<ChContactContainer> GetContactContainer() { return contact_container; }

    /// Given inserted markers and links, restores the
    /// pointers of links to markers given the information
    /// about the marker IDs. Will be made obsolete in future with new serialization systems.
    void Reference_LM_byID();

    //
    // STATISTICS
    //

    /// Gets the number of contacts.
    int GetNcontacts();

    /// Return the time (in seconds) spent for computing the time step.
    virtual double GetTimerStep() const { return timer_step(); }
    /// Return the time (in seconds) for time integration, within the time step.
    virtual double GetTimerAdvance() const { return timer_advance(); }
    /// Return the time (in seconds) for the solver, within the time step.
    /// Note that this time excludes any calls to the solver's Setup function.
    virtual double GetTimerSolver() const { return timer_solver(); }
    /// Return the time (in seconds) for the solver Setup phase, within the time step.
    virtual double GetTimerSetup() const { return timer_setup(); }
    /// Return the time (in seconds) for calculating/loading Jacobian information, within the time step.
    virtual double GetTimerJacobian() const { return timer_jacobian(); }
    /// Return the time (in seconds) for runnning the collision detection step, within the time step.
    virtual double GetTimerCollision() const { return timer_collision(); }
    /// Return the time (in seconds) for updating auxiliary data, within the time step.
    virtual double GetTimerUpdate() const { return timer_update(); }

    /// Return the time (in seconds) for broadphase collision detection, within the time step.
    double GetTimerCollisionBroad() const { return collision_system->GetTimerCollisionBroad(); }
    /// Return the time (in seconds) for narrowphase collision detection, within the time step.
    double GetTimerCollisionNarrow() const { return collision_system->GetTimerCollisionNarrow(); }

    /// Resets the timers.
    void ResetTimers() {
        timer_step.reset();
        timer_advance.reset();
        timer_solver.reset();
        timer_setup.reset();
        timer_jacobian.reset();
        timer_collision.reset();
        timer_update.reset();
        collision_system->ResetTimers();
    }

  protected:
    /// Pushes all ChConstraints and ChVariables contained in links, bodies, etc.
    /// into the system descriptor.
    virtual void DescriptorPrepareInject(ChSystemDescriptor& mdescriptor);

  public:
    //
    // PHYSICS ITEM INTERFACE
    //

    /// Counts the number of bodies and links.
    /// Computes the offsets of object states in the global state.
    /// Assumes that offset_x, offset_w, and offset_L are already set
    /// as starting point for offsetting all the contained sub objects.
    virtual void Setup() override;

    /// Updates all the auxiliary data and children of
    /// bodies, forces, links, given their current state.
    virtual void Update(bool update_assets = true) override;

    // (Overload interfaces for global state vectors, see ChPhysicsItem for comments.)
    // (The following must be overload because there may be ChContactContainer objects in addition to base ChAssembly)
    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) override;
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;

    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsLoadJacobians() override;

    virtual void InjectKRMmatrices(ChSystemDescriptor& mdescriptor) override;
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) override;

    // Old bookkeeping system 
    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesQbIncrementPosition(double step) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsBiLoad_Qc(double factor = 1) override;
    virtual void ConstraintsFbLoadForces(double factor = 1) override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    //
    // TIMESTEPPER INTERFACE
    //

    /// Tells the number of position coordinates x in y = {x, v}
    virtual int GetNcoords_x() override { return GetNcoords(); }

    /// Tells the number of speed coordinates of v in y = {x, v} and  dy/dt={v, a}
    virtual int GetNcoords_v() override { return GetNcoords_w(); }

    /// Tells the number of lagrangian multipliers (constraints)
    virtual int GetNconstr() override { return GetNdoc_w(); }

    /// From system to state y={x,v}
    virtual void StateGather(ChState& x, ChStateDelta& v, double& T) override;

    /// From state Y={x,v} to system.
    virtual void StateScatter(const ChState& x, const ChStateDelta& v, const double T) override;

    /// From system to state derivative (acceleration), some timesteppers might need last computed accel.
    virtual void StateGatherAcceleration(ChStateDelta& a) override;

    /// From state derivative (acceleration) to system, sometimes might be needed
    virtual void StateScatterAcceleration(const ChStateDelta& a) override;

    /// From system to reaction forces (last computed) - some timestepper might need this
    virtual void StateGatherReactions(ChVectorDynamic<>& L) override;

    /// From reaction forces to system, ex. store last computed reactions in ChLink objects for plotting etc.
    virtual void StateScatterReactions(const ChVectorDynamic<>& L) override;

    /// Perform x_new = x + dx    for x in    Y = {x, dx/dt}
    /// It takes care of the fact that x has quaternions, dx has angular vel etc.
    /// NOTE: the system is not updated automatically after the state increment, so one might
    /// need to call StateScatter() if needed.
    virtual void StateIncrementX(ChState& x_new,         ///< resulting x_new = x + Dx
                                 const ChState& x,       ///< initial state x
                                 const ChStateDelta& Dx  ///< state increment Dx
                                 ) override;

    /// Assuming a DAE of the form
    ///       M*a = F(x,v,t) + Cq'*L
    ///       C(x,t) = 0
    /// this function computes the solution of the change Du (in a or v or x) for a Newton
    /// iteration within an implicit integration scheme.
    ///  |Du| = [ G   Cq' ]^-1 * | R |
    ///  |DL|   [ Cq  0   ]      | Qc|
    /// for residual R and  G = [ c_a*M + c_v*dF/dv + c_x*dF/dx ]
    /// This function returns true if successful and false otherwise.
    virtual bool StateSolveCorrection(
        ChStateDelta& Dv,                 ///< result: computed Dv
        ChVectorDynamic<>& L,             ///< result: computed lagrangian multipliers, if any
        const ChVectorDynamic<>& R,       ///< the R residual
        const ChVectorDynamic<>& Qc,      ///< the Qc residual
        const double c_a,                 ///< the factor in c_a*M
        const double c_v,                 ///< the factor in c_v*dF/dv
        const double c_x,                 ///< the factor in c_x*dF/dv
        const ChState& x,                 ///< current state, x part
        const ChStateDelta& v,            ///< current state, v part
        const double T,                   ///< current time T
        bool force_state_scatter = true,  ///< if false, x,v and T are not scattered to the system
        bool force_setup = true           ///< if true, call the solver's Setup() function
        ) override;

    /// Increment a vector R with the term c*F:
    ///    R += c*F
    virtual void LoadResidual_F(ChVectorDynamic<>& R,  ///< result: the R residual, R += c*F
                                const double c         ///< a scaling factor
                                ) override;

    /// Increment a vector R with a term that has M multiplied a given vector w:
    ///    R += c*M*w
    virtual void LoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                 const ChVectorDynamic<>& w,  ///< the w vector
                                 const double c               ///< a scaling factor
                                 ) override;

    /// Increment a vectorR with the term Cq'*L:
    ///    R += c*Cq'*L
    virtual void LoadResidual_CqL(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                  const ChVectorDynamic<>& L,  ///< the L vector
                                  const double c               ///< a scaling factor
                                  ) override;

    /// Increment a vector Qc with the term C:
    ///    Qc += c*C
    virtual void LoadConstraint_C(ChVectorDynamic<>& Qc,        ///< result: the Qc residual, Qc += c*C
                                  const double c,               ///< a scaling factor
                                  const bool do_clamp = false,  ///< enable optional clamping of Qc
                                  const double mclam = 1e30     ///< clamping value
                                  ) override;

    /// Increment a vector Qc with the term Ct = partial derivative dC/dt:
    ///    Qc += c*Ct
    virtual void LoadConstraint_Ct(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*Ct
                                   const double c          ///< a scaling factor
                                   ) override;

    //
    // UTILITY FUNCTIONS
    //

    /// If ChProbe() objects are added to this system, using this command you force
    /// the ChProbe::Record() on all them, at once.
    int RecordAllProbes();

    /// If ChProbe() objects are added to this system, using this command you force
    /// the ChProbe::Reset() on all them, at once.
    int ResetAllProbes();

    /// Executes custom processing at the end of step. By default it does nothing,
    /// but if you inherit a special ChSystem you can implement this.
    virtual void CustomEndOfStep() {}

    /// If ChControl() objects are added to this system, using the following commands
    /// you call the execution of their scripts. You seldom call these functions directly,
    /// since the ChSystem() methods already call them automatically, at each step, update, etc.
    bool ExecuteControlsForUpdate();
    bool ExecuteControlsForStep();

    /// All bodies with collision detection data are requested to
    /// store the current position as "last position collision-checked"
    void SynchronizeLastCollPositions();

    /// Perform the collision detection.
    /// New contacts are inserted in the ChContactContainer object(s), and old ones are removed.
    /// This is mostly called automatically by time integration.
    double ComputeCollisions();

    /// Class to be used as a callback interface for user defined actions performed 
    /// at each collision detection step.  For example, additional contact points can
    /// be added to the underlying contact container.
    class ChApi CustomCollisionCallback {
      public:
        virtual ~CustomCollisionCallback() {}
        virtual void OnCustomCollision(ChSystem* msys) {}
    };

    /// Specify a callback object to be invoked at each collision detection step.
    /// Multiple such callback objects can be registered with a system. If present,
    /// their OnCustomCollision() method is invoked 
    /// Use this if you want that some specific callback function is executed at each
    /// collision detection step (ex. all the times that ComputeCollisions() is automatically
    /// called by the integration method). For example some other collision engine could
    /// add further contacts using this callback.
    void RegisterCustomCollisionCallback(CustomCollisionCallback* mcallb) {
        collision_callbacks.push_back(mcallb);
    }

    /// For higher performance (ex. when GPU coprocessors are available) you can create your own custom
    /// collision engine (inherited from ChCollisionSystem) and plug it into the system using this function. 
    /// Note: use only _before_ you start adding colliding bodies to the system!
    void SetCollisionSystem(std::shared_ptr<collision::ChCollisionSystem> newcollsystem);

    /// Access the collision system, the engine which
    /// computes the contact points (usually you don't need to
    /// access it, since it is automatically handled by the
    /// client ChSystem object).
    std::shared_ptr<collision::ChCollisionSystem> GetCollisionSystem() const { return collision_system; }

    /// Turn on this feature to let the system put to sleep the bodies whose
    /// motion has almost come to a rest. This feature will allow faster simulation
    /// of large scenarios for real-time purposes, but it will affect the precision!
    /// This functionality can be turned off selectively for specific ChBodies.
    void SetUseSleeping(bool ms) { use_sleeping = ms; }

    /// Tell if the system will put to sleep the bodies whose motion has almost come to a rest.
    bool GetUseSleeping() const { return use_sleeping; }

  private:
    /// Put bodies to sleep if possible. Also awakens sleeping bodies, if needed.
    /// Returns true if some body changed from sleep to no sleep or viceversa,
    /// returns false if nothing changed. In the former case, also performs Setup()
    /// because the sleeping policy changed the totalDOFs and offsets.
    bool ManageSleepingBodies();

    /// Performs a single dynamical simulation step, according to
    /// current values of:  Y, time, step  (and other minor settings)
    /// Depending on the integration type, it switches to one of the following:
    virtual bool Integrate_Y();

  public:
    // ---- DYNAMICS

    /// Advances the dynamical simulation for a single step, of
    /// length m_step. You can call this function many
    /// times in order to simulate up to a desired end time.
    /// This is the most important function for analysis, you
    /// can use it, for example, once per screen refresh in VR
    /// and interactive realtime applications, etc.
    int DoStepDynamics(double m_step);

    /// Performs integration until the m_endtime is exactly
    /// reached, but current time step may be automatically "retouched" to
    /// meet exactly the m_endtime after n steps.
    /// Useful when you want to advance the simulation in a
    /// simulations (3d modeling software etc.) which needs updates
    /// of the screen at a fixed rate (ex.30th of second)  while
    /// the integration must use more steps.
    bool DoFrameDynamics(double m_endtime);

    /// Given the current state, the sw simulates the
    /// dynamical behavior of the system, until the end
    /// time is reached, repeating many steps (maybe the step size
    /// will be automatically changed if the integrator method supports
    /// step size adaption).
    bool DoEntireDynamics();

    /// Like "DoEntireDynamics", but results are provided at uniform
    /// steps "frame_step", using the DoFrameDynamics() many times.
    bool DoEntireUniformDynamics(double frame_step);

    /// Return the total number of time steps taken so far.
    size_t GetStepcount() const { return stepcount; }

    /// Reset to 0 the total number of time steps.
    void ResetStepcount() { stepcount = 0; }

    /// Return the number of calls to the solver's Solve() function.
    /// This counter is reset at each timestep.
    int GetSolverCallsCount() const { return solvecount; }

    /// Return the number of calls to the solver's Setup() function.
    /// This counter is reset at each timestep.
    int GetSolverSetupCount() const { return setupcount; }

    /// Set this to "true" to enable automatic saving of solver matrices at each time
    /// step, for debugging purposes. Note that matrices will be saved in the
    /// working directory of the exe, with format 0001_01_H.dat 0002_01_H.dat
    /// (if the timestepper requires multiple solves, also 0001_01. 0001_02.. etc.)
    /// The matrices being saved are:
    ///    dump_Z.dat   has the assembled optimization matrix (Matlab sparse format)
    ///    dump_rhs.dat has the assembled RHS
    ///    dump_H.dat   has usually H=M (mass), but could be also H=a*M+b*K+c*R or such. (Matlab sparse format)
    ///    dump_Cq.dat  has the jacobians (Matlab sparse format)
    ///    dump_E.dat   has the constr.compliance (Matlab sparse format)
    ///    dump_f.dat   has the applied loads
    ///    dump_b.dat   has the constraint rhs
    /// as passed to the solver in the problem
    ///  | H -Cq'|*|q|- | f|= |0| , l \f$\in Y, c \in Ny\f$, normal cone to Y
    ///  | Cq -E | |l|  |-b|  |c|

    void SetDumpSolverMatrices(bool md) { dump_matrices = md; }
    bool GetDumpSolverMatrices() const { return dump_matrices; }

    /// Dump the current M mass matrix, K damping matrix, R damping matrix, Cq constraint jacobian
    /// matrix (at the current configuration). 
    /// These can be later used for linearized motion, modal analysis, buckling analysis, etc.
    /// The name of the files will be [path]_M.dat [path]_K.dat [path]_R.dat [path]_Cq.dat 
    /// Might throw ChException if file can't be saved.
    void DumpSystemMatrices(bool save_M, bool save_K, bool save_R, bool save_Cq, const char* path);

    /// Compute the system-level mass matrix. 
    /// This function has a small overhead, because it must assembly the
    /// sparse matrix -which is used only for the purpose of this function.
    void GetMassMatrix(ChSparseMatrix* M);    ///< fill this system mass matrix

    /// Compute the system-level stiffness matrix, i.e. the jacobian -dF/dq where F are stiff loads.
    /// Note that not all loads provide a jacobian, as this is optional in their implementation.
    /// This function has a small overhead, because it must assembly the
    /// sparse matrix -which is used only for the purpose of this function.
    void GetStiffnessMatrix(ChSparseMatrix* K);    ///< fill this system stiffness matrix

    /// Compute the system-level damping matrix, i.e. the jacobian -dF/dv where F are stiff loads.
    /// Note that not all loads provide a jacobian, as this is optional in their implementation.
    /// This function has a small overhead, because it must assembly the
    /// sparse matrix -which is used only for the purpose of this function.
    void GetDampingMatrix(ChSparseMatrix* R);    ///< fill this system damping matrix

    /// Compute the system-level constraint jacobian matrix, i.e. the jacobian
    /// Cq=-dC/dq where C are constraints (the lower left part of the KKT matrix).
    /// This function has a small overhead, because it must assembly the
    /// sparse matrix -which is used only for the purpose of this function.
    void GetConstraintJacobianMatrix(ChSparseMatrix* Cq);  ///< fill this system damping matrix

    // ---- KINEMATICS

    /// Advances the kinematic simulation for a single step, of
    /// length m_step. You can call this function many
    /// times in order to simulate up to a desired end time.
    bool DoStepKinematics(double m_step);

    /// Performs kinematics until the m_endtime is exactly
    /// reached, but current time step may be automatically "retouched" to
    /// meet exactly the m_endtime after n steps.
    bool DoFrameKinematics(double m_endtime);

    /// Given the current state, this kinematic simulation
    /// satisfies all the constraints with the "DoStepKinematics"
    /// procedure for each time step, from the current time
    /// to the end time.
    bool DoEntireKinematics();

    // ---- CONSTRAINT ASSEMBLATION

    /// Given the current time and state, attempt to satisfy all constraints, using
    /// a Newton-Raphson iteration loop. Used iteratively in inverse kinematics.
    /// Action can be one of AssemblyLevel::POSITION, AssemblyLevel::VELOCITY, or 
    /// AssemblyLevel::ACCELERATION (or a combination of these)
    /// Returns true if no errors and false if an error occurred (impossible assembly?)
    bool DoAssembly(int action);

    /// Shortcut for full position/velocity/acceleration assembly.
    bool DoFullAssembly();

    // ---- STATICS

    /// Solve the position of static equilibrium (and the
    /// reactions). This is a one-step only approach that solves
    /// the _linear_ equilibrium. To be used mostly for FEM
    /// problems with small deformations.
    bool DoStaticLinear();

    /// Solve the position of static equilibrium (and the
    /// reactions). This tries to solve the equilibrium for the nonlinear
    /// problem (large displacements). The larger nsteps, the more the CPU time
    /// but the less likely the divergence.
    bool DoStaticNonlinear(int nsteps = 10);

    /// Finds the position of static equilibrium (and the
    /// reactions) starting from the current position.
    /// Since a truncated iterative method is used, you may need
    /// to call this method multiple times in case of large nonlinearities
    /// before coming to the precise static solution.
    bool DoStaticRelaxing(int nsteps = 10);

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

    /// Process a ".chr" binary file containing the full system object
    /// hierarchy as exported -for example- by the R3D modeler, with chrono plug-in version,
    /// or by using the FileWriteChR() function.
    int FileProcessChR(ChStreamInBinary& m_file);

    /// Write a ".chr" binary file containing the full system object
    /// hierarchy (bodies, forces, links, etc.) (deprecated function - obsolete)
    int FileWriteChR(ChStreamOutBinary& m_file);

  protected:
    std::vector<std::shared_ptr<ChProbe> > probelist;        ///< list of 'probes' (variable-recording objects)
    std::vector<std::shared_ptr<ChControls> > controlslist;  ///< list of 'controls' script objects

    std::shared_ptr<ChContactContainer> contact_container;  ///< the container of contacts

    ChVector<> G_acc;  ///< gravitational acceleration

    double end_time;  ///< end of simulation
    double step;      ///< time step
    double step_min;  ///< min time step
    double step_max;  ///< max time step

    double tol;        ///< tolerance
    double tol_force;  ///< tolerance for forces (used to obtain a tolerance for impulses)

    int maxiter;  ///< max iterations for nonlinear convergence in DoAssembly()

    bool use_sleeping;  ///< if true, put to sleep objects that come to rest

    std::shared_ptr<ChSystemDescriptor> descriptor;  ///< the system descriptor
    std::shared_ptr<ChSolver> solver_speed;          ///< the solver for speed problem
    std::shared_ptr<ChSolver> solver_stab;           ///< the solver for position (stabilization) problem, if any

    int max_iter_solver_speed;  ///< maximum num iterations for the iterative solver
    int max_iter_solver_stab;   ///< maximum num iterations for the iterative solver for constraint stabilization
    int max_steps_simplex;      ///< maximum number of steps for the simplex solver.

    double min_bounce_speed;                ///< minimum speed for rebounce after impacts. Lower speeds are clamped to 0
    double max_penetration_recovery_speed;  ///< limit for the speed of penetration recovery (positive, speed of exiting)

    int parallel_thread_number;  ///< used for multithreaded solver

    size_t stepcount;  ///< internal counter for steps

    int setupcount;  ///< number of calls to the solver's Setup()
    int solvecount;  ///< number of StateSolveCorrection (reset to 0 at each timestep of static analysis)

    bool dump_matrices;  ///< for debugging

    int ncontacts;  ///< total number of contacts

    std::shared_ptr<collision::ChCollisionSystem> collision_system;  ///< collision engine

    std::vector<CustomCollisionCallback*> collision_callbacks;

    std::unique_ptr<ChMaterialCompositionStrategy<float>> composition_strategy; /// material composition strategy

    // timers for profiling execution speed
    ChTimer<double> timer_step;       ///< timer for integration step
    ChTimer<double> timer_advance;    ///< timer for time integration
    ChTimer<double> timer_solver;     ///< timer for solver (excluding setup phase)
    ChTimer<double> timer_setup;      ///< timer for solver setup
    ChTimer<double> timer_jacobian;   ///< timer for computing/loading Jacobian information
    ChTimer<double> timer_collision;  ///< timer for collision detection
    ChTimer<double> timer_update;     ///< timer for system update

    std::shared_ptr<ChTimestepper> timestepper;  ///< time-stepper object

    bool last_err;  ///< indicates error over the last kinematic/dynamics/statics

    // Friend class declarations

    template <class Ta, class Tb>
    friend class ChContactNSC;

    template <class Ta, class Tb>
    friend class ChContactNSCrolling;

    template <class Ta, class Tb>
    friend class ChContactSMC;
};

CH_CLASS_VERSION(ChSystem, 0)

}  // end namespace chrono

#endif
