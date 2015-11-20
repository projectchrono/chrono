//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSYSTEM_H
#define CHSYSTEM_H


//   The physical system definition.
//   A phisical system encloses bodies, links,
//   probes, etc.


#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>
#include <list>

#include "core/ChLog.h"
#include "core/ChMath.h"
#include "core/ChTimer.h"
#include "physics/ChAssembly.h"
#include "physics/ChLinksAll.h"
#include "physics/ChEvents.h"
#include "physics/ChProbe.h"
#include "physics/ChControls.h"
#include "physics/ChMaterialCouple.h"
#include "physics/ChScriptEngine.h"
#include "physics/ChGlobal.h"
#include "physics/ChContactContainerBase.h"
#include "collision/ChCCollisionSystem.h"
#include "timestepper/ChIntegrable.h"
#include "timestepper/ChTimestepper.h"

namespace chrono {

// forward references & shortcuts..

typedef ChSharedPtr<ChLink> ChSharedLinkPtr;
typedef ChSharedPtr<ChProbe> ChSharedProbePtr;
typedef ChSharedPtr<ChControls> ChSharedControlsPtr;
class ChLcpSolver;
class ChLcpSystemDescriptor;
class ChContactContainerBase;





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

class ChApi ChSystem : public ChAssembly, 
                       public ChIntegrableIIorderEasy {
    CH_RTTI(ChSystem, ChAssembly);

  public:
    //
    // BUILDERS
    //

    /// Create a physical system.
    /// Note, in case you will use collision detection, the values of
    /// 'max_objects' and 'scene_size' can be used to initialize the broadphase
    /// collision algorithm in an optimal way. Scene size should be approximately
    /// the radius of the expected area where colliding objects will move.
    /// Note that currently, by default, the collision broadphase is a btDbvtBroadphase
    /// that does not make use of max_objects and scene_size, but one might plug-in
    /// other collision engines that might use those parameters.
    /// If init_sys is false it does not initialize the collision system or solver
    /// assumes that the user will do so.
    ChSystem(unsigned int max_objects = 16000, double scene_size = 500, bool init_sys = true);

    /// Destructor
    virtual ~ChSystem();
    /// Copy from another ChSystem.
    /// Note! All settings are copied, but the hierarchy of children
    /// bodies, links, probes etc. are NOT copied.
    void Copy(ChSystem* source);

    //
    // PROPERTIES
    //

    /// Sets the time step used for integration (dynamical simulation).
    /// The lower this value, the more precise the simulation. Usually, values
    /// about 0.01 s are enough for simple simulations. It may be modified automatically
    /// by integration methods, if they support automatic time adaption.
    void SetStep(double m_step) {
        if (m_step > 0.)
            step = m_step;
    }
    /// Gets the current time step used for the integration (dynamical simulation).
    double GetStep() { return step; }

    /// Sets the end of simulation.
    void SetEndTime(double m_end_time) { end_time = m_end_time; }
    /// Gets the end of the simulation
    double GetEndTime() { return end_time; }

    /// Sets the lower limit for time step (only needed if using
    /// integration methods which support time step adaption).
    void SetStepMin(double m_step_min) {
        if (m_step_min > 0.)
            step_min = m_step_min;
    }
    /// Gets the lower limit for time step
    double GetStepMin() { return step_min; }

    /// Sets the upper limit for time step (only needed if using
    /// integration methods which support time step adaption).
    void SetStepMax(double m_step_max) {
        if (m_step_max > step_min)
            step_max = m_step_max;
    }
    /// Gets the upper limit for time step
    double GetStepMax() { return step_max; }

    /// Available methods for time integration (time steppers).
    enum eCh_integrationType {
        INT_ANITESCU = 0,
        INT_TASORA = 6,
        INT_EULER_IMPLICIT = 7,
        INT_EULER_IMPLICIT_LINEARIZED = 8,
        INT_EULER_IMPLICIT_PROJECTED = 17,
        INT_TRAPEZOIDAL = 9,
        INT_TRAPEZOIDAL_LINEARIZED = 10,
        INT_HHT = 11,
        INT_HEUN = 12,
        INT_RUNGEKUTTA45 = 13,
        INT_EULER_EXPLICIT = 14,
        INT_LEAPFROG = 15,
        INT_NEWMARK = 16,
        INT_CUSTOM__ = 17,
    };
    CH_ENUM_MAPPER_BEGIN(eCh_integrationType);
      CH_ENUM_VAL(INT_ANITESCU);
      CH_ENUM_VAL(INT_TASORA);
      CH_ENUM_VAL(INT_EULER_IMPLICIT);
      CH_ENUM_VAL(INT_EULER_IMPLICIT_LINEARIZED);
      CH_ENUM_VAL(INT_EULER_IMPLICIT_PROJECTED);
      CH_ENUM_VAL(INT_TRAPEZOIDAL);
      CH_ENUM_VAL(INT_TRAPEZOIDAL_LINEARIZED);
      CH_ENUM_VAL(INT_HHT);
      CH_ENUM_VAL(INT_HEUN);
      CH_ENUM_VAL(INT_RUNGEKUTTA45);
      CH_ENUM_VAL(INT_EULER_EXPLICIT);
      CH_ENUM_VAL(INT_LEAPFROG);
      CH_ENUM_VAL(INT_NEWMARK);
      CH_ENUM_VAL(INT_CUSTOM__);
    CH_ENUM_MAPPER_END(eCh_integrationType);

    /// Sets the method for time integration (time stepper).
    /// Suggested for fast dynamics with hard (DVI) contacts: INT_EULER_IMPLICIT_LINEARIZED,
    /// Suggested for fast dynamics with hard (DVI) contacts and low inter-penetration: INT_EULER_IMPLICIT_PROJECTED,
    /// Suggested for finite element smooth dynamics: INT_HHT, INT_EULER_IMPLICIT_LINEARIZED.
    /// NOTE: for more advanced customization, use SetTimestepper().
    void SetIntegrationType(eCh_integrationType m_integration_type);

    /// Gets the current method for time integration (time stepper).
    eCh_integrationType GetIntegrationType() { return integration_type; }


    /// Set the timestepper to be used for time integration.
    /// This is more powerful than SetIntegrationType, because you can provide your own object.
    /// Also sets the mode to INT_CUSTOM__ , should you ever call GetIntegrationType() later.
    void SetTimestepper(ChSharedPtr<ChTimestepper> mstepper) { this->timestepper = mstepper; integration_type = INT_CUSTOM__; }

    /// Get the timestepper currently used for time integration
    ChSharedPtr<ChTimestepper> GetTimestepper() { return this->timestepper; }


    /// Sets outer iteration limit for assembly constraints. When trying to keep constraints together,
    /// the iterative process is stopped if this max.number of iterations (or tolerance) is reached.
    void SetMaxiter(int m_maxiter) { maxiter = m_maxiter; }
    /// Gets iteration limit for assembly constraints.
    int GetMaxiter() { return maxiter; }

    /// Sets tolerance (in m) for assembly constraints. When trying to keep constraints together,
    /// the iterative process is stopped if this tolerance (or max.number of iterations ) is reached
    void SetTol(double m_tol) { tol = m_tol; }
    /// Gets current tolerance for assembly constraints.
    double GetTol() { return tol; }

    /// Sets tolerance for satisfying constraints at the velocity level.
    /// The tolerance specified here is in fact a tolerance at the force level.
    /// this value is multiplied by the value of the current time step and then
    /// used as a stopping criteria for the iterative speed solver.
    void SetTolForce(double mtol) { tol_force = mtol; }

    /// Return the current value of the tolerance used in the speed solver.
    double GetTolForce() const { return tol_force; }


    /// For elastic collisions, with objects that have nonzero
    /// restitution coefficient: objects will rebounce only if their
    /// relative colliding speed is above this threshold. Default 0.15 m/s.
    /// If this is too low, aliasing problems can happen with small high frequency
    /// rebounces, and settling to static stacking might be more difficult.
    void SetMinBounceSpeed(double mval) { min_bounce_speed = mval; }
    /// Objects will rebounce only if their relative colliding speed is above this threshold.
    double GetMinBounceSpeed() { return min_bounce_speed; }

    /// For the Anitescu stepper, you can limit the speed of exiting from penetration
    /// situations. Usually set a positive value, about 0.1 .. 2 . (as exiting speed, in m/s)
    void SetMaxPenetrationRecoverySpeed(double mval) { max_penetration_recovery_speed = mval; }
    /// Get the limit on the speed for exiting from penetration situations (for Anitescu stepper)
    double GetMaxPenetrationRecoverySpeed() { return max_penetration_recovery_speed; }

    /// Available types of solvers for the LCP problem. Note: compared to iterative methods,
    /// the simplex solver is so slow that it's mostly for experiments.
    /// Also, Jacobi is slower than SOR - hence it's here for benchmarks & tests.
    enum eCh_lcpSolver {
        LCP_ITERATIVE_SOR = 0,
        LCP_ITERATIVE_SYMMSOR,
        LCP_SIMPLEX,  // OBSOLETE!
        LCP_ITERATIVE_JACOBI,
        LCP_ITERATIVE_SOR_MULTITHREAD,
        LCP_ITERATIVE_PMINRES,
        LCP_ITERATIVE_BARZILAIBORWEIN,
        LCP_ITERATIVE_PCG,
        LCP_ITERATIVE_APGD,
        LCP_DEM,
        LCP_ITERATIVE_MINRES,
        LCP_CUSTOM,
    };
    CH_ENUM_MAPPER_BEGIN(eCh_lcpSolver);
      CH_ENUM_VAL(LCP_ITERATIVE_SOR);
      CH_ENUM_VAL(LCP_ITERATIVE_SYMMSOR);
      CH_ENUM_VAL(LCP_SIMPLEX);
      CH_ENUM_VAL(LCP_ITERATIVE_JACOBI);
      CH_ENUM_VAL(LCP_ITERATIVE_SOR_MULTITHREAD);
      CH_ENUM_VAL(LCP_ITERATIVE_PMINRES);
      CH_ENUM_VAL(LCP_ITERATIVE_BARZILAIBORWEIN);
      CH_ENUM_VAL(LCP_ITERATIVE_PCG);
      CH_ENUM_VAL(LCP_ITERATIVE_APGD);
      CH_ENUM_VAL(LCP_DEM);
      CH_ENUM_VAL(LCP_ITERATIVE_MINRES);
      CH_ENUM_VAL(LCP_CUSTOM);
    CH_ENUM_MAPPER_END(eCh_lcpSolver);

    /// Choose the LCP solver type, to be used for the simultaneous
    /// solution of the constraints in dynamical simulations (as well as
    /// in kinematics, statics, etc.)
    /// You can choose between the eCh_lcpSolver types, ex. LCP_ITERATIVE_SOR
    /// for speed and low precision, LCP_ITERATIVE_BARZILAIBORWEIN for precision, etc.
    /// NOTE: Do not use LCP_CUSTOM, this type will be set automatically set if one 
    /// provides its solver via ChangeLcpSolverStab etc.
    /// NOTE: This is a shortcut, that internally is equivalent to the two
    ///  calls ChangeLcpSolverStab(..) and ChangeLcpSolverSpeed(...)
    virtual void SetLcpSolverType(eCh_lcpSolver mval);
    /// Gets the current LCP solver type.
    eCh_lcpSolver GetLcpSolverType() { return lcp_solver_type; }

    /// In case you are using an iterative LCP solver (es. LCP_ITERATIVE_SOR)
    /// you can set the maximum number of iterations. The higher the
    /// iteration number, the more precise the simulation (but more CPU time)
    void SetIterLCPmaxItersSpeed(int mval) { iterLCPmaxIters = mval; }
    /// Current maximum number of iterations, if using an iterative LCP solver.
    int GetIterLCPmaxItersSpeed() { return iterLCPmaxIters; }

    /// In case you are using an iterative LCP solver (es. LCP_ITERATIVE_SOR)
    /// and an integration method requiring post-stabilization (es. INT_TASORA)
    /// you can set the maximum number of stabilization iterations. The higher the
    /// iteration number, the more precise the simulation (but more CPU time)
    int GetIterLCPmaxItersStab() { return iterLCPmaxItersStab; }
    /// Current maxi. number of iterations, if using an iterative LCP solver for stabilization.
    void SetIterLCPmaxItersStab(int mval) { iterLCPmaxItersStab = mval; }


    /// If you want to easily turn ON/OFF the warm starting feature of both LCP iterative solvers
    /// (the one for speed and the other for pos.stabilization) you can simply use the
    /// following instead of accessing them directly with GetLcpSolverSpeed() and GetLcpSolverStab()
    void SetIterLCPwarmStarting(bool usewarm = true);
    /// Tell if the warm starting is enabled for the speed solver, (if iterative type).
    bool GetIterLCPwarmStarting();

    /// If you want to easily adjust the omega overrelaxation parameter of both LCP iterative solvers
    /// (the one for speed and the other for pos.stabilization) you can simply use the
    /// following instead of accessing them directly with GetLcpSolverSpeed() and GetLcpSolverStab().
    /// Note, usually a good omega for Jacobi or GPU solver is 0.2; for other iter.solvers can be up to 1.0
    void SetIterLCPomega(double momega = 1.0);
    /// Tell the omega overrelaxation factor for the speed solver, (if iterative type).
    double GetIterLCPomega();

    /// If you want to easily adjust the 'sharpness lambda' parameter of both LCP iterative solvers
    /// (the one for speed and the other for pos.stabilization) you can simply use the
    /// following instead of accessing them directly with GetLcpSolverSpeed() and GetLcpSolverStab().
    /// Note, usually a good sharpness value is in 1..0.8 range (the lower, the more it helps exact
    /// convergence, but overall convergence gets also much slower so maybe better to tolerate some error)
    void SetIterLCPsharpnessLambda(double momega = 1.0);
    /// Tell the 'sharpness lambda' factor for the speed solver, (if iterative type).
    double GetIterLCPsharpnessLambda();

    /// Instead of using SetLcpSolverType(), you can create your own
    /// custom lcp solver (suffice it is inherited from ChLcpSolver) and plug
    /// it into the system using this function. The replaced solver is automatically deleted.
    /// When the system is deleted, the custom solver that you plugged will be automatically deleted.
    /// Note: this also sets the LCP_CUSTOM mode, should you ever call GetLcpSolverType() later.
    virtual void ChangeLcpSolverStab(ChLcpSolver* newsolver);

    /// Access directly the LCP solver, configured to be used for the stabilization
    /// of constraints (solve delta positions). Use mostly for diagnostics.
    virtual ChLcpSolver* GetLcpSolverStab();

    /// Instead of using SetLcpSolverType(), you can create your own
    /// custom lcp solver (suffice it is inherited from ChLcpSolver) and plug
    /// it into the system using this function. The replaced solver is automatically deleted.
    /// When the system is deleted, the custom solver that you plugged will be automatically deleted.
    /// Note: this also sets the LCP_CUSTOM mode, should you ever call GetLcpSolverType() later.
    virtual void ChangeLcpSolverSpeed(ChLcpSolver* newsolver);

    /// Access directly the LCP solver, configured to be used for the main differential
    /// inclusion problem (LCP on speed-impulses). Use mostly for diagnostics.
    virtual ChLcpSolver* GetLcpSolverSpeed();

    /// Instead of using the default LCP 'system descriptor', you can create your own
    /// custom descriptor (suffice it is inherited from ChLcpSystemDescriptor) and plug
    /// it into the system using this function. The replaced descriptor is automatically deleted.
    /// When the system is deleted, the custom descriptor that you plugged will be automatically deleted.
    void ChangeLcpSystemDescriptor(ChLcpSystemDescriptor* newdescriptor);

    /// Access directly the LCP 'system descriptor'. Use mostly for diagnostics.
    ChLcpSystemDescriptor* GetLcpSystemDescriptor() { return this->LCP_descriptor; };

    /// Changes the number of parallel threads (by default is n.of cores).
    /// Note that not all solvers use parallel computation.
    /// If you have a N-core processor, this should be set at least =N for maximum performance.
    void SetParallelThreadNumber(int mthreads = 2);
    /// Get the number of parallel threads.
    /// Note that not all solvers use parallel computation.
    int GetParallelThreadNumber() { return parallel_thread_number; }

    /// Sets the G (gravity) acceleration vector, affecting all the bodies in the system.
    void Set_G_acc(ChVector<> m_acc = ChVector<>(0.0, -9.8, 0.0)) { G_acc = m_acc; }
    /// Gets the G (gravity) acceleration vector affecting all the bodies in the system.
    ChVector<> Get_G_acc() { return G_acc; }


    //
    // DATABASE HANDLING.
    //

    /// Removes all bodies/marker/forces/links/contacts,
    /// also resets timers and events.
    void Clear();

    /// Return the contact method supported by this system.
    /// Bodies added to this system must be compatible.
    virtual ChMaterialSurfaceBase::ContactMethod GetContactMethod() const { return ChMaterialSurfaceBase::DVI; }

    /// Create and return the pointer to a new body.
    /// The returned body is created with a contact model consistent with the type
    /// of this Chsystem and with the collision system currently associated with this
    /// ChSystem.  Note that the body is *not* attached to this system.
    virtual ChBody* NewBody() { return new ChBody(ChMaterialSurfaceBase::DVI); }

    /// Attach a probe to this system.
    void AddProbe(ChSharedPtr<ChProbe>& newprobe);
    /// Attach a control to this system.
    void AddControls(ChSharedPtr<ChControls>& newcontrols);

    /// Remove all probes from this system.
    void RemoveAllProbes();
    /// Remove all controls from this system.
    void RemoveAllControls();


    /// For higher performance (ex. when GPU coprocessors are available) you can create your own
    /// custom contact container (suffice it is inherited from ChContactContainerBase) and plug
    /// it into the system using this function. The replaced container is automatically deleted.
    /// When the system is deleted, the custom container that you plugged will be automatically deleted.
    virtual void ChangeContactContainer(ChSharedPtr<ChContactContainerBase> newcontainer);

    /// Get the contact container
    ChSharedPtr<ChContactContainerBase> GetContactContainer() { return contact_container; }


    /// Given inserted markers and links, restores the
    /// pointers of links to markers given the information
    /// about the marker IDs. Will be made obsolete in future with new serialization systems.
    void Reference_LM_byID();

    //
    // STATISTICS
    //

    
    /// Gets the number of contacts.
    int GetNcontacts();

    /// Gets the time (in seconds) spent for computing the time step
    virtual double GetTimerStep() { return timer_step(); }
    /// Gets the fraction of time (in seconds) for the solution of the LCPs, within the time step
    virtual double GetTimerLcp() { return timer_lcp(); }
    /// Gets the fraction of time (in seconds) for finding collisions, within the time step
    virtual double GetTimerCollisionBroad() { return timer_collision_broad(); }
    /// Gets the fraction of time (in seconds) for finding collisions, within the time step
    virtual double GetTimerCollisionNarrow() { return timer_collision_narrow(); }
    /// Gets the fraction of time (in seconds) for updating auxiliary data, within the time step
    virtual double GetTimerUpdate() { return timer_update(); }

    /// Resets the timers.
    void ResetTimers() {
        timer_step.reset();
        timer_lcp.reset();
        timer_collision_broad.reset();
        timer_collision_narrow.reset();
        timer_update.reset();
    }

    
    /// Gets the cyclic event buffer of this system (it can be used for
    /// debugging/profiling etc.)
    ChEvents* Get_events() { return events; }

  protected:
    //
    // LCP SOLVER
    //

    /// Sets to zero all the known terms bi & fb of the sparse LCP (that is,
    /// resets all the bi terms in ChConstraints (for example constraints
    /// defined in ChLinks, and resets all the fb vectors of ChVariables
    /// contained, for example, in ChBodies)
    virtual void LCPprepare_reset();

    /// Fills the all the known terms of the sparse LCP (that is,
    /// fills all the bi terms in ChConstraints (for example constraints
    /// defined in ChLinks, and fills all the fb vectors of ChVariables
    /// contained, for example, in ChBodies).
    /// The parameters of this function specify which data must be loaded
    /// in the known terms.
    virtual void LCPprepare_load(
        bool load_jacobians,  ///< load jacobians into ChConstraints
        bool load_Mv,         ///< load M*v in fb: fb+=M*v (for timestepping where fb=F*h+M*v_old). Also, sets q=v_old.
        double F_factor,      ///< load F (forces) in fb: fb+=F*F_factor
        double K_factor,      ///< load K stiff.matrices, if any ChLcpKblock matrices, multiplied by K_factor
        double R_factor,      ///< load R damp.matrices, if any ChLcpKblock matrices, multiplied by R_factor
        double M_factor,      ///< load M mass.matrices, if any ChLcpKblock matrices, multiplied by M_factor (ex in
        /// non-lumped-mass FEM)
        double Ct_factor,       ///< load Ct into bi:  bi+= Ct*Ct_factor
        double C_factor,        ///< load C  into bi:  bi+= C*C_factor, otherwise..
        double recovery_clamp,  ///< if do_clamp=true,  bi+= min(C*C_factor, recovery_clamp);
        bool do_clamp           ///< if true, limit the recovery of constraint drifting
        );

    /// Pushes back all ChConstraints and ChVariables contained in links,bodies,etc.
    /// into the LCP descriptor.
    virtual void LCPprepare_inject(ChLcpSystemDescriptor& mdescriptor);

    /// The following constraints<->system functions are used before and after the solution of a LCP, because
    /// iterative LCP solvers may converge faster to the Li lagrangian multiplier solutions if 'guessed'
    /// values provided (exploit the fact that ChLink classes implement caches with 'last computed multipliers').
    virtual void LCPprepare_Li_from_speed_cache();
    virtual void LCPprepare_Li_from_position_cache();
    virtual void LCPresult_Li_into_speed_cache();
    virtual void LCPresult_Li_into_position_cache();
    virtual void LCPresult_Li_into_reactions(double mfactor);

public:
    //
    // PHYSICS ITEM INTERFACE
    //


    /// Counts the number of bodies and links.
    /// Computes the offsets of object states in the global state.
    /// Assumes that this->offset_x this->offset_w this->offset_L are already set
    /// as starting point for offsetting all the contained sub objects.
    virtual void Setup();

    /// Updates all the auxiliary data and children of
    /// bodies, forces, links, given their current state.
    virtual void Update(bool update_assets = true);


    // (Overload interfaces for global state vectors, see ChPhysicsItem for comments.)
    // (The following must be overload because there may be ChContactContainer objects in addition to base ChAssembly)
    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T);
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T);
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a);
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a);
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L);
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L);
    virtual void IntStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv);
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c);
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c);
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c);
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp);
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c);
    virtual void IntToLCP(const unsigned int off_v,
                          const ChStateDelta& v,
                          const ChVectorDynamic<>& R,
                          const unsigned int off_L,
                          const ChVectorDynamic<>& L,
                          const ChVectorDynamic<>& Qc);
    virtual void IntFromLCP(const unsigned int off_v, 
                            ChStateDelta& v, 
                            const unsigned int off_L, 
                            ChVectorDynamic<>& L);
    
    virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);
    
    virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
    virtual void ConstraintsLoadJacobians();

    virtual void InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor);
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor);

    // Old bookkeeping system - to be removed soon
    virtual void VariablesFbReset();
    virtual void VariablesFbLoadForces(double factor = 1.);
    virtual void VariablesQbLoadSpeed();
    virtual void VariablesFbIncrementMq();
    virtual void VariablesQbSetSpeed(double step = 0.);
    virtual void VariablesQbIncrementPosition(double step);
    virtual void ConstraintsBiReset();
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false);
    virtual void ConstraintsBiLoad_Ct(double factor = 1.);
    virtual void ConstraintsBiLoad_Qc(double factor = 1.);
    virtual void ConstraintsFbLoadForces(double factor = 1.);
    
    virtual void ConstraintsLiLoadSuggestedSpeedSolution();
    virtual void ConstraintsLiLoadSuggestedPositionSolution();
    virtual void ConstraintsLiFetchSuggestedSpeedSolution();
    virtual void ConstraintsLiFetchSuggestedPositionSolution();
    virtual void ConstraintsFetch_react(double factor = 1.);

    //
    // TIMESTEPPER INTERFACE
    //

    /// Tells the number of position coordinates x in y = {x, v}
    virtual int GetNcoords_x() { return this->GetNcoords(); }

    /// Tells the number of speed coordinates of v in y = {x, v} and  dy/dt={v, a}
    virtual int GetNcoords_v() { return this->GetNcoords_w(); }

    /// Tells the number of lagrangian multipliers (constraints)
    virtual int GetNconstr() { return this->GetNdoc_w(); }

    /// From system to state y={x,v}
    virtual void StateGather(ChState& x, ChStateDelta& v, double& T);

    /// From state Y={x,v} to system.
    virtual void StateScatter(const ChState& x, const ChStateDelta& v, const double T);

    /// From system to state derivative (acceleration), some timesteppers might need last computed accel.
    virtual void StateGatherAcceleration(ChStateDelta& a);

    /// From state derivative (acceleration) to system, sometimes might be needed
    virtual void StateScatterAcceleration(const ChStateDelta& a);

    /// From system to reaction forces (last computed) - some timestepper might need this
    virtual void StateGatherReactions(ChVectorDynamic<>& L);

    /// From reaction forces to system, ex. store last computed reactions in ChLink objects for plotting etc.
    virtual void StateScatterReactions(const ChVectorDynamic<>& L);

    /// Perform x_new = x + dx    for x in    Y = {x, dx/dt}
    /// It takes care of the fact that x has quaternions, dx has angular vel etc.
    /// NOTE: the system is not updated automatically after the state increment, so one might
    /// need to call StateScatter() if needed.
    virtual void StateIncrementX(ChState& x_new,         ///< resulting x_new = x + Dx
                                 const ChState& x,       ///< initial state x
                                 const ChStateDelta& Dx  ///< state increment Dx
                                 );

    /// Assuming an explicit DAE in the form
    ///        M*a = F(x,v,t) + Cq'*L
    ///       C(x,t) = 0
    /// this must compute the solution of the change Du (in a or v or x) to satisfy
    /// the equation required in a Newton Raphson iteration for an
    /// implicit integrator equation:
    ///  |Du| = [ G   Cq' ]^-1 * | R |
    ///  |DL|   [ Cq  0   ]      | Qc|
    /// for residual R and  G = [ c_a*M + c_v*dF/dv + c_x*dF/dx ]
    virtual void StateSolveCorrection(ChStateDelta& Dv,             ///< result: computed Dv
                                      ChVectorDynamic<>& L,         ///< result: computed lagrangian multipliers, if any
                                      const ChVectorDynamic<>& R,   ///< the R residual
                                      const ChVectorDynamic<>& Qc,  ///< the Qc residual
                                      const double c_a,             ///< the factor in c_a*M
                                      const double c_v,             ///< the factor in c_v*dF/dv
                                      const double c_x,             ///< the factor in c_x*dF/dv
                                      const ChState& x,             ///< current state, x part
                                      const ChStateDelta& v,        ///< current state, v part
                                      const double T,               ///< current time T
                                      bool force_state_scatter = true  ///< if false, x,v and T are not scattered to the
                                      /// system, assuming that someone has done
                                      /// StateScatter just before
                                      );

    /// Increment a vector R with the term c*F:
    ///    R += c*F
    virtual void LoadResidual_F(ChVectorDynamic<>& R,  ///< result: the R residual, R += c*F
                                const double c         ///< a scaling factor
                                );

    /// Increment a vector R with a term that has M multiplied a given vector w:
    ///    R += c*M*w
    virtual void LoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                 const ChVectorDynamic<>& w,  ///< the w vector
                                 const double c               ///< a scaling factor
                                 );

    /// Increment a vectorR with the term Cq'*L:
    ///    R += c*Cq'*L
    virtual void LoadResidual_CqL(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                  const ChVectorDynamic<>& L,  ///< the L vector
                                  const double c               ///< a scaling factor
                                  );

    /// Increment a vector Qc with the term C:
    ///    Qc += c*C
    virtual void LoadConstraint_C(ChVectorDynamic<>& Qc,        ///< result: the Qc residual, Qc += c*C
                                  const double c,               ///< a scaling factor
                                  const bool do_clamp = false,  ///< enable optional clamping of Qc
                                  const double mclam = 1e30     ///< clamping value
                                  );

    /// Increment a vector Qc with the term Ct = partial derivative dC/dt:
    ///    Qc += c*Ct
    virtual void LoadConstraint_Ct(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*Ct
                                   const double c          ///< a scaling factor
                                   );

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
    virtual void CustomEndOfStep(){};

    /// Set the script engine (ex. a Javascript engine).
    /// The user must take care of creating and deleting the script
    /// engine , if any, and deletion must happen after deletion of the ChSystem.
    void SetScriptEngine(ChScriptEngine* mengine) { this->scriptEngine = mengine; }
    ChScriptEngine* GetScriptEngine() { return this->scriptEngine; }

    const std::string& GetScriptForStartFile() { return scriptForStartFile; }
    const std::string& GetScriptForUpdateFile() { return scriptForUpdateFile; }
    const std::string& GetScriptForStepFile() { return scriptForStepFile; }
    const std::string& GetScriptFor3DStepFile() { return scriptFor3DStepFile; }
    int SetScriptForStartFile(const std::string& mfile);
    int SetScriptForUpdateFile(const std::string& mfile);
    int SetScriptForStepFile(const std::string& mfile);
    int SetScriptFor3DStepFile(const std::string& mfile);
    int ExecuteScriptForStart();
    int ExecuteScriptForUpdate();
    int ExecuteScriptForStep();
    int ExecuteScriptFor3DStep();

    /// If ChControl() objects are added to this system, using the following commands
    /// you call the execution of their scripts. You seldom call these functions directly,
    /// since the ChSystem() methods already call them automatically, at each step, update, etc.
    int ExecuteControlsForStart();
    int ExecuteControlsForUpdate();
    int ExecuteControlsForStep();
    int ExecuteControlsFor3DStep();

    /// All bodies with collision detection data are requested to
    /// store the current position as "last position collision-checked"
    void SynchronizeLastCollPositions();

    /// Perform the collision detection.
    /// New contacts are inserted in the ChContactContainer object(s), and
    /// old are removed.
    /// This is mostly called automatically by time integration.
    double ComputeCollisions();

    /// Class to be inherited by user and to use in SetCustomComputeCollisionCallback()
    class ChApi ChCustomComputeCollisionCallback {
      public:
        virtual void PerformCustomCollision(ChSystem* msys){};
    };
    /// Use this if you want that some specific callback function is executed at each
    /// collision detection step (ex. all the times that ComputeCollisions() is automatically
    /// called by the integration method). For example some other collision engine could
    /// add further contacts using this callback.
    void SetCustomComputeCollisionCallback(ChCustomComputeCollisionCallback* mcallb) { collision_callbacks.push_back(mcallb); };

    /// Class to be inherited by user and to use in SetCustomCollisionPointCallback()
    class ChApi ChCustomCollisionPointCallback {
      public:
        virtual void ContactCallback(
            const collision::ChCollisionInfo& mcontactinfo,  ///< get info about contact (cannot change it)
            ChMaterialCouple& material                       ///< you can modify this!
            ) = 0;
    };
    /// Use this if you want that some specific callback function is executed soon after
    /// each contact point is created. The callback will be called many times, once for each contact.
    /// Example: it can be used to modify the friction coefficients for each created
    /// contact (otherwise, by default, would be the average of the two frict.coeff.)
    void SetCustomCollisionPointCallback(ChCustomCollisionPointCallback* mcallb) { collisionpoint_callback = mcallb; };


    /// For higher performance (ex. when GPU coprocessors are available) you can create your own
    /// custom collision engine (suffice it is inherited from ChCollisionSystem) and plug
    /// it into the system using this function. The replaced engine is automatically deleted.
    /// When the system is deleted, the custom engine that you plugged will be automatically deleted.
    /// Note: use only _before_ you start adding colliding bodies to the system!
    void ChangeCollisionSystem(collision::ChCollisionSystem* newcollsystem);

    /// Access the collision system, the engine which
    /// computes the contact points (usually you don't need to
    /// access it, since it is automatically handled by the
    /// client ChSystem object).
    collision::ChCollisionSystem* GetCollisionSystem() { return collision_system; };

    /// Turn on this feature to let the system put to sleep the bodies whose
    /// motion has almost come to a rest. This feature will allow faster simulation
    /// of large scenarios for real-time purposes, but it will affect the precision!
    /// This functionality can be turned off selectively for specific ChBodies.
    void SetUseSleeping(bool ms) { use_sleeping = ms; }

    /// Tell if the system will put to sleep the bodies whose motion has almost come to a rest.
    bool GetUseSleeping() { return use_sleeping; }

  private:

    /// Put bodies to sleep if possible. Also awakens sleeping bodies, if needed.
    /// Returns true if some body changed from sleep to no sleep or viceversa, 
    /// returns false if nothing changed. In the former case, also performs Setup() 
    /// because the sleeping policy changed the totalDOFs and offsets.
    bool ManageSleepingBodies();

    //
    // ANALYSIS FUNCTIONS
    //

  private:
    /// Performs a single dynamical simulation step, according to
    /// current values of:  Y, time, step  (and other minor settings)
    /// Depending on the integration type, it switches to one of the following:
    virtual int Integrate_Y();

    /// Use Anitescu stepper, with position stabilization in speed stage.
    virtual int Integrate_Y_impulse_Anitescu();

    /// Use Tasora stepper, with separate stage for position stabilization.
    virtual int Integrate_Y_impulse_Tasora();

    /// Use the new pluggable ChTimestepper
    virtual int Integrate_Y_timestepper();

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
    /// simulations (3d modeling software etc.) wihch needs updates
    /// of the screen at a fixed rate (ex.30th of second)  while
    /// the integration must use more steps.
    int DoFrameDynamics(double m_endtime);

    /// Given the current state, the sw simulates the
    /// dynamical behaviour of the system, until the end
    /// time is reached, repeating many steps (maybe the step size
    /// will be automatically changed if the integrator method supports
    /// step size adaption).
    int DoEntireDynamics();

    /// Like "DoEntireDynamics", but results are provided at uniform
    /// steps "frame_step", using the DoFrameDynamics() many times.
    int DoEntireUniformDynamics(double frame_step);

    /// Return the total amount of performed time steps
    size_t GetStepcount() {return stepcount;}
    /// Reset to 0 the total amount of performed time steps
    void ResetStepcount() {stepcount = 0;}

    /// Return the number of StateSolveCorrection operations 
    /// (reset to 0 at each timestep of static analysis)
    int GetSolverCallsCount() { return solvecount;}

    /// Set this to "true" to enable saving of matrices at each time
    /// step, for debugging purposes. Note that matrices will be saved in the
    /// working directory of the exe, with format 0001_01_M.dat 0002_01_M.dat 
    /// (if the timestepper requires multiple solves, also 0001_01. 0001_02.. etc.)
    void SetDumpMatrices(bool md) { dump_matrices = md;}
    bool GetDumpMatrices() { return dump_matrices;}

    // ---- KINEMATICS

    /// Advances the kinematic simulation for a single step, of
    /// length m_step. You can call this function many
    /// times in order to simulate up to a desired end time.
    int DoStepKinematics(double m_step);

    /// Performs kinematics until the m_endtime is exactly
    /// reached, but current time step may be automatically "retouched" to
    /// meet exactly the m_endtime after n steps.
    int DoFrameKinematics(double m_endtime);

    /// Given the current state, this kinematic simulation
    /// satisfies all the costraints with the "DoStepKinematics"
    /// procedure for each time step, from the current time
    /// to the end time.
    int DoEntireKinematics();

    // ---- CONSTRAINT ASSEMBLATION

    /// Given the current time and state, the
    /// sw tries to satisfy all costraints, with
    /// the Newton-Raphson iteration. Used iteratively
    /// in inverse kinematics.
    /// Different tolerance checking allowable (norm 1/2/inf)
    ///  mode = [action flags, see above], ASS_POSITION , ASS_SPEED , ASS_ACCEL (also together)
    ///  flags = [see above]
    ///  ASF_COLLISION , perform also collision detection
    /// Returns 0 if no errors, returns TRUE if error happened (impossible assemblation?)
    int DoAssembly(int action, int mflags = 0);

    /// Shortcut for full pos/speed/acc assembly, also computes forces
    int DoFullAssembly();

    // ---- STATICS

    /// Solve the position of static equilibrium (and the
    /// reactions). This is a one-step only approach that solves
    /// the _linear_ equilibrium. To be used mostly for FEM
    /// problems with small deformations.
    int DoStaticLinear();

    /// Solve the position of static equilibrium (and the
    /// reactions). This tries to solve the equilibrium for the nonlinear
    /// problem (large displacements). The larger nsteps, the more the CPU time
    /// but the less likely the divergence.
    int DoStaticNonlinear(int nsteps = 10);

    /// Finds the position of static equilibrium (and the
    /// reactions) starting from the current position.
    /// Since a truncated iterative metod is used, you may need
    /// to call this method multiple times in case of large nonlienarities
    /// before coming to the precise static solution.
    int DoStaticRelaxing(int nsteps =10);

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);

    //
    // STREAMING
    //


    /// Process a ".chr" binary file containing the full system object
    /// hierarchy as exported -for example- by the R3D modeler, with chrono plugin version,
    /// or by using the FileWriteChR() function.
    int FileProcessChR(ChStreamInBinary& m_file);

    /// Write a ".chr" binary file containing the full system object
    /// hierarchy (bodies, forces, links, etc.) (deprecated function - obsolete)
    int FileWriteChR(ChStreamOutBinary& m_file);



    /////////////////////////////////////////////////////////////////////////////////////

  protected:
    //
    // DATA
    //

    // list of 'probes' (variable-recording objects, exp. for
    // 3rd party apps)
    std::vector< ChSharedPtr<ChProbe> > probelist;

    // list of 'controls' script objects (objects containing
    // scripting programs and GUI panels, exp. for 3rd party apps)
    std::vector< ChSharedPtr<ChControls> > controlslist;

    // the container of contacts
    ChSharedPtr< ChContactContainerBase > contact_container;

    ChVector<> G_acc;  // gravitational acceleration

    double end_time;  // end of simulation, in seconds
    double step;      // time step, in seconds
    double step_min;  // min time step
    double step_max;  // max time step

    double tol;        // tolerance
    double tol_force;  // tolerance for forces (used to obtain a tolerance for impulses)

    int maxiter;       // max iterations for nonlinear convergence in DoAssembly()

    bool use_sleeping;   // if true, can put to sleep objects that come to rest, to speed up simulation (but decreasing
                         // the precision)

    eCh_integrationType integration_type;  // integration scheme

    ChLcpSystemDescriptor* LCP_descriptor;  // the LCP system descriptor
    ChLcpSolver* LCP_solver_speed;          // the LCP solver for speed problem
    ChLcpSolver* LCP_solver_stab;           // the LCP solver for position (stabilization) problem, if any
    eCh_lcpSolver lcp_solver_type;  // Type of LCP solver (iterative= fastest, but may fail satisfying constraints)

    int iterLCPmaxIters;      // maximum n.of iterations for the iterative LCP solver
    int iterLCPmaxItersStab;  // maximum n.of iterations for the iterative LCP solver when used for stabilizing
                              // constraints
    int simplexLCPmaxSteps;   // maximum number of steps for the simplex solver.

    double min_bounce_speed;  // maximum speed for rebounce after impacts. If lower speed at rebounce, it is clamped to
                              // zero.
    double max_penetration_recovery_speed;  // For Anitescu/Eulero impl. linearized stepper, this limits the speed of penetration recovery
                                            // (>0, speed of exiting)

    int parallel_thread_number;  // used for multithreaded solver etc.

    size_t stepcount;  // internal counter for steps

    int solvecount; // number of StateSolveCorrection (reset to 0 at each timestep os static analysis)

    bool dump_matrices; // for debugging

    int ncontacts; 

    // The collision engine, to compute and store contact manifolds
    collision::ChCollisionSystem* collision_system;

    std::vector<ChCustomComputeCollisionCallback*> collision_callbacks;

  public:
    ChCustomCollisionPointCallback* collisionpoint_callback;

  private:
    int last_err;                    // If null, no error during last kinematic/dynamics/statics etc.
                                     // otherwise see CHSYS_ERR_xxxx  code

    ChEvents* events;  // the cyclic buffer which records event IDs

    ChScriptEngine* scriptEngine;  // points to a script engine
    ChScript* scriptForStart;      // this script is executed when simulation starts.
    std::string scriptForStartFile;
    ChScript* scriptForUpdate;  // this script is executed for each Update step.
    std::string scriptForUpdateFile;
    ChScript* scriptForStep;  // this script is executed for each integration step
    std::string scriptForStepFile;
    ChScript* scriptFor3DStep;  // this script is executed for each 3d interface macro step
    std::string scriptFor3DStepFile;

    // timers for profiling execution speed
  protected:
    ChTimer<double> timer_step;
    ChTimer<double> timer_lcp;
    ChTimer<double> timer_collision_broad;
    ChTimer<double> timer_collision_narrow;
    ChTimer<double> timer_update;

    ChSharedPtr<ChTimestepper> timestepper;
};

//////////////////////////////////////
// Define flags for "action" of
// DoAssembly()  function

#define ASS_POSITION (1L << 0)
#define ASS_SPEED (1L << 1)
#define ASS_ACCEL (1L << 2)

// define other flags for "flags"
// argument of DoAssembly() function
#define ASF_NONE 0
#define ASF_COLLISIONS (1L << 6)

}  // END_OF_NAMESPACE____

#endif  // ond of ChSystem.h
