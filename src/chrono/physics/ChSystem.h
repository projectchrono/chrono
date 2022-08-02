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

#include "chrono/core/ChGlobal.h"
#include "chrono/core/ChLog.h"
#include "chrono/core/ChMath.h"
#include "chrono/core/ChTimer.h"
#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/utils/ChOpenMP.h"
#include "chrono/physics/ChAssembly.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChSolver.h"
#include "chrono/timestepper/ChAssemblyAnalysis.h"
#include "chrono/timestepper/ChIntegrable.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/timestepper/ChTimestepperHHT.h"
#include "chrono/timestepper/ChStaticAnalysis.h"

namespace chrono {

// Forward references
class ChVisualSystem;
namespace modal {
class ChModalAssembly;
}

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

class ChApi ChSystem : public ChIntegrableIIorder {

  public:
    /// Create a physical system.
    ChSystem();

    /// Copy constructor
    ChSystem(const ChSystem& other);

    /// Destructor
    virtual ~ChSystem();

    /// "Virtual" copy constructor.
    /// Concrete derived classes must implement this.
    virtual ChSystem* Clone() const = 0;

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

    /// Change the default composition laws for contact surface materials
    /// (coefficient of friction, cohesion, compliance, etc.)
    virtual void SetMaterialCompositionStrategy(std::unique_ptr<ChMaterialCompositionStrategy>&& strategy);

    /// Accessor for the current composition laws for contact surface material.
    const ChMaterialCompositionStrategy& GetMaterialCompositionStrategy() const { return *composition_strategy; }

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

    /// Attach a solver (derived from ChSolver) for use by this system.
    virtual void SetSolver(std::shared_ptr<ChSolver> newsolver);

    /// Access the solver currently associated with this system.
    virtual std::shared_ptr<ChSolver> GetSolver();

    /// Choose the solver type, to be used for the simultaneous solution of the constraints
    /// in dynamical simulations (as well as in kinematics, statics, etc.)
    ///   - Suggested solver for speed, but lower precision: PSOR
    ///   - Suggested solver for higher precision: BARZILAIBORWEIN or APGD
    ///   - For problems that involve a stiffness matrix: GMRES, MINRES
    ///
    /// *Notes*:
    ///   - This function is a shortcut, internally equivalent to a call to SetSolver().
    ///   - Only a subset of available Chrono solvers can be set through this mechanism.
    ///   - Prefer explicitly creating a solver, setting solver parameters, and then attaching the solver with
    ///     SetSolver.
    ///
    /// \deprecated This function does not support all available Chrono solvers. Prefer using SetSolver.
    void SetSolverType(ChSolver::Type type);

    /// Gets the current solver type.
    ChSolver::Type GetSolverType() const { return solver->GetType(); }

    /// Set the maximum number of iterations, if using an iterative solver.
    /// \deprecated Prefer using SetSolver and setting solver parameters directly.
    void SetSolverMaxIterations(int max_iters);

    /// Get the current maximum number of iterations, if using an iterative solver.
    /// \deprecated Prefer using GetSolver and accessing solver statistics directly.
    int GetSolverMaxIterations() const;

    /// Set the solver tolerance threshold (used with iterative solvers only).
    /// Note that the stopping criteria differs from solver to solver.
    void SetSolverTolerance(double tolerance);

    /// Get the current tolerance value (used with iterative solvers only).
    double GetSolverTolerance() const;

    /// Set a solver tolerance threshold at force level (default: not specified).
    /// Specify this value **only** if solving the problem at velocity level (e.g. solving a DVI problem).
    /// If this tolerance is specified, it is multiplied by the current integration stepsize and overwrites the current
    /// solver tolerance.  By default, this tolerance is invalid and hence the solver's own tolerance threshold is used.
    void SetSolverForceTolerance(double tolerance) { tol_force = tolerance; }

    /// Get the current value of the force-level tolerance (used with iterative solvers only).
    double GetSolverForceTolerance() const { return tol_force; }

    /// Instead of using the default 'system descriptor', you can create your own custom descriptor
    /// (inherited from ChSystemDescriptor) and plug it into the system using this function.
    void SetSystemDescriptor(std::shared_ptr<ChSystemDescriptor> newdescriptor);

    /// Access directly the 'system descriptor'.
    std::shared_ptr<ChSystemDescriptor> GetSystemDescriptor() { return descriptor; }

    /// Set the G (gravity) acceleration vector, affecting all the bodies in the system.
    void Set_G_acc(const ChVector<>& m_acc) { G_acc = m_acc; }

    /// Get the G (gravity) acceleration vector affecting all the bodies in the system.
    const ChVector<>& Get_G_acc() const { return G_acc; }

    /// Get the simulation time of this system.
    double GetChTime() const { return ch_time; }

    /// Set (overwrite) the simulation time of this system.
    void SetChTime(double time) { ch_time = time; }

    /// Set the number of OpenMP threads used by Chrono itself, Eigen, and the collision detection system.
    /// <pre>
    ///   num_threads_chrono    - used in FEA (parallel evaluation of internal forces and Jacobians) and
    ///                           in SCM deformable terrain calculations.
    ///   num_threads_collision - used in parallelization of collision detection (if applicable).
    ///                           If passing 0, then num_threads_collision = num_threads_chrono.
    ///   num_threads_eigen     - used in the Eigen sparse direct solvers and a few linear algebra operations.
    ///                           Note that Eigen enables multi-threaded execution only under certain size conditions.
    ///                           See the Eigen documentation.
    ///                           If passing 0, then num_threads_eigen = num_threads_chrono.
    /// By default (if this function is not called), the following values are used:
    ///   num_threads_chrono = omp_get_num_procs()
    ///   num_threads_collision = 1
    ///   num_threads_eigen = 1
    /// </pre>
    /// Note that a derived class may ignore some or all of these settings.
    virtual void SetNumThreads(int num_threads_chrono, int num_threads_collision = 0, int num_threads_eigen = 0);

    int GetNumThreadsChrono() const { return nthreads_chrono; }
    int GetNumthreadsCollision() const { return nthreads_collision; }
    int GetNumthreadsEigen() const { return nthreads_eigen; }

    //
    // DATABASE HANDLING
    //

    /// Get the underlying assembly containing all physics items.
    const ChAssembly& GetAssembly() const { return assembly; }

    /// Attach a body to the underlying assembly.
    virtual void AddBody(std::shared_ptr<ChBody> body);

    /// Attach a shaft to the underlying assembly.
    virtual void AddShaft(std::shared_ptr<ChShaft> shaft);

    /// Attach a link to the underlying assembly.
    virtual void AddLink(std::shared_ptr<ChLinkBase> link);

    /// Attach a mesh to the underlying assembly.
    virtual void AddMesh(std::shared_ptr<fea::ChMesh> mesh);

    /// Attach a ChPhysicsItem object that is not a body, link, or mesh.
    virtual void AddOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item);

    /// Attach an arbitrary ChPhysicsItem (e.g. ChBody, ChParticles, ChLink, etc.) to the assembly.
    /// It will take care of adding it to the proper list of bodies, links, meshes, or generic physic item. (i.e. it
    /// calls AddBody, AddShaft(), AddLink(), AddMesh(), or AddOtherPhysicsItem()). Note, you cannot call Add() during
    /// an Update (i.e. items like particle generators that are already inserted in the assembly cannot call this)
    /// because not thread safe; instead, use AddBatch().
    void Add(std::shared_ptr<ChPhysicsItem> item);

    /// Items added in this way are added like in the Add() method, but not instantly,
    /// they are simply queued in a batch of 'to add' items, that are added automatically
    /// at the first Setup() call. This is thread safe.
    void AddBatch(std::shared_ptr<ChPhysicsItem> item) { assembly.AddBatch(item); }

    /// If some items are queued for addition in the assembly, using AddBatch(), this will
    /// effectively add them and clean the batch. Called automatically at each Setup().
    void FlushBatch() { assembly.FlushBatch(); }

    /// Remove a body from this assembly.
    virtual void RemoveBody(std::shared_ptr<ChBody> body) { assembly.RemoveBody(body); }

    /// Remove a shaft from this assembly.
    virtual void RemoveShaft(std::shared_ptr<ChShaft> shaft) { assembly.RemoveShaft(shaft); }

    /// Remove a link from this assembly.
    virtual void RemoveLink(std::shared_ptr<ChLinkBase> link) { assembly.RemoveLink(link); }

    /// Remove a mesh from the assembly.
    virtual void RemoveMesh(std::shared_ptr<fea::ChMesh> mesh) { assembly.RemoveMesh(mesh); }

    /// Remove a ChPhysicsItem object that is not a body or a link
    virtual void RemoveOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item) { assembly.RemoveOtherPhysicsItem(item); }

    /// Remove arbitrary ChPhysicsItem that was added to the underlying assembly.
    void Remove(std::shared_ptr<ChPhysicsItem> item);

    /// Remove all bodies from the underlying assembly.
    void RemoveAllBodies() { assembly.RemoveAllBodies(); }
    /// Remove all shafts from the underlying assembly.
    void RemoveAllShafts() { assembly.RemoveAllShafts(); }
    /// Remove all links from the underlying assembly.
    void RemoveAllLinks() { assembly.RemoveAllLinks(); }
    /// Remove all meshes from the underlying assembly.
    void RemoveAllMeshes() { assembly.RemoveAllMeshes(); }
    /// Remove all physics items not in the body, link, or mesh lists.
    void RemoveAllOtherPhysicsItems() { assembly.RemoveAllOtherPhysicsItems(); }

    /// Get the list of bodies.
    const std::vector<std::shared_ptr<ChBody>>& Get_bodylist() const { return assembly.bodylist; }
    /// Get the list of shafts.
    const std::vector<std::shared_ptr<ChShaft>>& Get_shaftlist() const { return assembly.shaftlist; }
    /// Get the list of links.
    const std::vector<std::shared_ptr<ChLinkBase>>& Get_linklist() const { return assembly.linklist; }
    /// Get the list of meshes.
    const std::vector<std::shared_ptr<fea::ChMesh>>& Get_meshlist() const { return assembly.meshlist; }
    /// Get the list of physics items that are not in the body or link lists.
    const std::vector<std::shared_ptr<ChPhysicsItem>>& Get_otherphysicslist() const {
        return assembly.otherphysicslist;
    }

    /// Search a body by its name.
    std::shared_ptr<ChBody> SearchBody(const char* name) { return assembly.SearchBody(name); }
    /// Search a body by its ID
    std::shared_ptr<ChBody> SearchBodyID(int bodyID) { return assembly.SearchBodyID(bodyID); }
    /// Search a shaft by its name.
    std::shared_ptr<ChShaft> SearchShaft(const char* name) { return assembly.SearchShaft(name); }
    /// Search a link by its name.
    std::shared_ptr<ChLinkBase> SearchLink(const char* name) { return assembly.SearchLink(name); }
    /// Search a mesh by its name.
    std::shared_ptr<fea::ChMesh> SearchMesh(const char* name) { return assembly.SearchMesh(name); }
    /// Search from other ChPhysics items (not bodies, links, or meshes) by name.
    std::shared_ptr<ChPhysicsItem> SearchOtherPhysicsItem(const char* name) {
        return assembly.SearchOtherPhysicsItem(name);
    }
    /// Search a marker by its name.
    std::shared_ptr<ChMarker> SearchMarker(const char* name) { return assembly.SearchMarker(name); }
    /// Search a marker by its unique ID.
    std::shared_ptr<ChMarker> SearchMarker(int markID) { return assembly.SearchMarker(markID); }
    /// Search an item (body, link or other ChPhysics items) by name.
    std::shared_ptr<ChPhysicsItem> Search(const char* name) { return assembly.Search(name); }

    /// Get the number of active bodies (excluding those that are sleeping or are fixed to ground).
    int GetNbodies() const { return assembly.GetNbodies(); }
    /// Get the number of bodies that are in sleeping mode (excluding fixed bodies).
    int GetNbodiesSleeping() const { return assembly.GetNbodiesSleeping(); }
    /// Get the number of bodies that are fixed to ground.
    int GetNbodiesFixed() const { return assembly.GetNbodiesFixed(); }
    /// Get the total number of bodies in the assembly, including the grounded and sleeping bodies.
    int GetNbodiesTotal() const { return assembly.GetNbodiesTotal(); }

    /// Get the number of shafts.
    int GetNshafts() const { return assembly.GetNshafts(); }
    /// Get the number of shafts that are in sleeping mode (excluding fixed shafts).
    int GetNshaftsSleeping() const { return assembly.GetNbodiesSleeping(); }
    /// Get the number of shafts that are fixed to ground.
    int GetNshaftsFixed() const { return assembly.GetNshaftsFixed(); }
    /// Get the total number of shafts added to the assembly, including the grounded and sleeping shafts.
    int GetNshaftsTotal() const { return assembly.GetNshaftsTotal(); }

    /// Get the number of links.
    int GetNlinks() const { return assembly.GetNlinks(); }

    /// Get the number of meshes.
    int GetNmeshes() const { return assembly.GetNmeshes(); }

    /// Get the number of other physics items (other than bodies, links, or meshes).
    int GetNphysicsItems() const { return assembly.GetNphysicsItems(); }

    /// Get the number of coordinates (considering 7 coords for rigid bodies because of the 4 dof of quaternions).
    int GetNcoords() const { return ncoords; }
    /// Get the number of degrees of freedom of the assembly.
    int GetNdof() const { return ndof; }
    /// Get the number of scalar constraints added to the assembly, including constraints on quaternion norms.
    int GetNdoc() const { return ndoc; }
    /// Get the number of system variables (coordinates plus the constraint multipliers, in case of quaternions).
    int GetNsysvars() const { return nsysvars; }
    /// Get the number of coordinates (considering 6 coords for rigid bodies, 3 transl.+3rot.)
    int GetNcoords_w() const { return ncoords_w; }
    /// Get the number of scalar constraints added to the assembly.
    int GetNdoc_w() const { return ndoc_w; }
    /// Get the number of scalar constraints added to the assembly (only bilaterals).
    int GetNdoc_w_C() const { return ndoc_w_C; }
    /// Get the number of scalar constraints added to the assembly (only unilaterals).
    int GetNdoc_w_D() const { return ndoc_w_D; }
    /// Get the number of system variables (coordinates plus the constraint multipliers).
    int GetNsysvars_w() const { return nsysvars_w; }

    /// Get the number of scalar coordinates (ex. dim of position vector)
    int GetDOF() const { return GetNcoords(); }
    /// Get the number of scalar coordinates of variables derivatives (ex. dim of speed vector)
    int GetDOF_w() const { return GetNcoords_w(); }
    /// Get the number of scalar constraints, if any, in this item
    int GetDOC() const { return GetNdoc_w(); }
    /// Get the number of scalar constraints, if any, in this item (only bilateral constr.)
    int GetDOC_c() const { return GetNdoc_w_C(); }
    /// Get the number of scalar constraints, if any, in this item (only unilateral constr.)
    int GetDOC_d() const { return GetNdoc_w_D(); }

    /// Write the hierarchy of contained bodies, markers, etc. in ASCII
    /// readable form, mostly for debugging purposes. Level is the tab spacing at the left.
    void ShowHierarchy(ChStreamOutAscii& m_file, int level = 0) const { assembly.ShowHierarchy(m_file, level); }

    /// Removes all bodies/marker/forces/links/contacts, also resets timers and events.
    void Clear();

    /// Return the contact method supported by this system.
    /// Contactables (bodies, FEA nodes, FEA traiangles, etc.) added to this system must be compatible.
    virtual ChContactMethod GetContactMethod() const = 0;

    /// Create and return the pointer to a new body.
    /// The body is consistent with the type of the collision system currently associated with this ChSystem.
    /// Note that the body is *not* attached to this system.
    virtual ChBody* NewBody();

    /// Create and return the pointer to a new body with auxiliary reference frame.
    /// The body is consistent with the type of the collision system currently associated with this ChSystem.
    /// Note that the body is *not* attached to this system.
    virtual ChBodyAuxRef* NewBodyAuxRef();

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
    virtual double GetTimerLSsolve() const { return timer_ls_solve(); }
    /// Return the time (in seconds) for the solver Setup phase, within the time step.
    virtual double GetTimerLSsetup() const { return timer_ls_setup(); }
    /// Return the time (in seconds) for calculating/loading Jacobian information, within the time step.
    virtual double GetTimerJacobian() const { return timer_jacobian(); }
    /// Return the time (in seconds) for runnning the collision detection step, within the time step.
    virtual double GetTimerCollision() const { return timer_collision(); }
    /// Return the time (in seconds) for system setup, within the time step.
    virtual double GetTimerSetup() const { return timer_setup(); }
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
        timer_ls_solve.reset();
        timer_ls_setup.reset();
        timer_jacobian.reset();
        timer_collision.reset();
        timer_setup.reset();
        timer_update.reset();
        collision_system->ResetTimers();
    }

  protected:
    /// Pushes all ChConstraints and ChVariables contained in links, bodies, etc. into the system descriptor.
    virtual void DescriptorPrepareInject(ChSystemDescriptor& mdescriptor);

    // Note: SetupInitial need not be typically called by a user, so it is currently marked protected
    // (as it may need to be called by derived classes)

    /// Initial system setup before analysis.
    /// This function performs an initial system setup, once system construction is completed and before an analysis.
    void SetupInitial();

    /// Return the resultant applied force on the specified body.
    /// This resultant force includes all external applied loads acting on the body (from gravity, loads, springs,
    /// etc). However, this does *not* include any constraint forces. In particular, contact forces are not included if
    /// using the NSC formulation, but are included when using the SMC formulation.
    virtual ChVector<> GetBodyAppliedForce(ChBody* body);

    /// Return the resultant applied torque on the specified body.
    /// This resultant torque includes all external applied loads acting on the body (from gravity, loads, springs,
    /// etc). However, this does *not* include any constraint forces. In particular, contact torques are not included if
    /// using the NSC formulation, but are included when using the SMC formulation.
    virtual ChVector<> GetBodyAppliedTorque(ChBody* body);

  public:
    /// Counts the number of bodies and links.
    /// Computes the offsets of object states in the global state. Assumes that offset_x, offset_w, and offset_L are
    /// already set as starting point for offsetting all the contained sub objects.
    virtual void Setup();

    /// Updates all the auxiliary data and children of
    /// bodies, forces, links, given their current state.
    void Update(double mytime, bool update_assets = true);

    /// Updates all the auxiliary data and children of bodies, forces, links, given their current state.
    void Update(bool update_assets = true);

    /// In normal usage, no system update is necessary at the beginning of a new dynamics step (since an update is
    /// performed at the end of a step). However, this is not the case if external changes to the system are made. Most
    /// such changes are discovered automatically (addition/removal of items, input of mesh loads). For special cases,
    /// this function allows the user to trigger a system update at the beginning of the step immediately following this
    /// call.
    void ForceUpdate();

    void IntToDescriptor(const unsigned int off_v,
                         const ChStateDelta& v,
                         const ChVectorDynamic<>& R,
                         const unsigned int off_L,
                         const ChVectorDynamic<>& L,
                         const ChVectorDynamic<>& Qc);
    void IntFromDescriptor(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L);

    void InjectVariables(ChSystemDescriptor& mdescriptor);

    void InjectConstraints(ChSystemDescriptor& mdescriptor);
    void ConstraintsLoadJacobians();

    void InjectKRMmatrices(ChSystemDescriptor& mdescriptor);
    void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor);

    // Old bookkeeping system
    void VariablesFbReset();
    void VariablesFbLoadForces(double factor = 1);
    void VariablesQbLoadSpeed();
    void VariablesFbIncrementMq();
    void VariablesQbSetSpeed(double step = 0);
    void VariablesQbIncrementPosition(double step);
    void ConstraintsBiReset();
    void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false);
    void ConstraintsBiLoad_Ct(double factor = 1);
    void ConstraintsBiLoad_Qc(double factor = 1);
    void ConstraintsFbLoadForces(double factor = 1);
    void ConstraintsFetch_react(double factor = 1);

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

    /// From state Y={x,v} to system. This also triggers an update operation.
    virtual void StateScatter(const ChState& x,
                              const ChStateDelta& v,
                              const double T,
                              bool full_update) override;

    /// From system to state derivative (acceleration), some timesteppers might need last computed accel.
    virtual void StateGatherAcceleration(ChStateDelta& a) override;

    /// From state derivative (acceleration) to system, sometimes might be needed
    virtual void StateScatterAcceleration(const ChStateDelta& a) override;

    /// From system to reaction forces (last computed) - some timestepper might need this
    virtual void StateGatherReactions(ChVectorDynamic<>& L) override;

    /// From reaction forces to system, ex. store last computed reactions in ChLink objects for plotting etc.
    virtual void StateScatterReactions(const ChVectorDynamic<>& L) override;

    /// Perform x_new = x + dx, for x in Y = {x, dx/dt}.\n
    /// It takes care of the fact that x has quaternions, dx has angular vel etc.
    /// NOTE: the system is not updated automatically after the state increment, so one might
    /// need to call StateScatter() if needed.
    virtual void StateIncrementX(ChState& x_new,         ///< resulting x_new = x + Dx
                                 const ChState& x,       ///< initial state x
                                 const ChStateDelta& Dx  ///< state increment Dx
                                 ) override;

    /// Assuming a DAE of the form
    /// <pre>
    ///       M*a = F(x,v,t) + Cq'*L
    ///       C(x,t) = 0
    /// </pre>
    /// this function computes the solution of the change Du (in a or v or x) for a Newton
    /// iteration within an implicit integration scheme.
    /// <pre>
    ///  |Du| = [ G   Cq' ]^-1 * | R |
    ///  |DL|   [ Cq  0   ]      | Qc|
    /// </pre>
    /// for residual R and  G = [ c_a*M + c_v*dF/dv + c_x*dF/dx ].\n
    /// This function returns true if successful and false otherwise.
    virtual bool StateSolveCorrection(
        ChStateDelta& Dv,             ///< result: computed Dv
        ChVectorDynamic<>& L,         ///< result: computed lagrangian multipliers, if any
        const ChVectorDynamic<>& R,   ///< the R residual
        const ChVectorDynamic<>& Qc,  ///< the Qc residual
        const double c_a,             ///< the factor in c_a*M
        const double c_v,             ///< the factor in c_v*dF/dv
        const double c_x,             ///< the factor in c_x*dF/dv
        const ChState& x,             ///< current state, x part
        const ChStateDelta& v,        ///< current state, v part
        const double T,               ///< current time T
        bool force_state_scatter,     ///< if false, x and v are not scattered to the system
        bool full_update,             ///< if true, perform a full update during scatter
        bool force_setup              ///< if true, call the solver's Setup() function
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
                                  const double clamp = 1e30     ///< clamping value
                                  ) override;

    /// Increment a vector Qc with the term Ct = partial derivative dC/dt:
    ///    Qc += c*Ct
    virtual void LoadConstraint_Ct(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*Ct
                                   const double c          ///< a scaling factor
                                   ) override;

    //
    // UTILITY FUNCTIONS
    //

    /// Executes custom processing at the end of step. By default it does nothing,
    /// but if you inherit a special ChSystem you can implement this.
    virtual void CustomEndOfStep() {}

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
    /// their OnCustomCollision() method is invoked.
    /// Use this if you want that some specific callback function is executed at each
    /// collision detection step (ex. all the times that ComputeCollisions() is automatically
    /// called by the integration method). For example some other collision engine could
    /// add further contacts using this callback.
    void RegisterCustomCollisionCallback(std::shared_ptr<CustomCollisionCallback> callback);

    /// Remove the given collision callback from this system.
    void UnregisterCustomCollisionCallback(std::shared_ptr<CustomCollisionCallback> callback);

    /// Change the underlying collision detection system to the specified type.
    /// By default, a ChSystem uses a Bullet-based collision detection engine
    /// (collision::ChCollisionSystemType::BULLET).
    virtual void SetCollisionSystemType(collision::ChCollisionSystemType type);

    /// Change the underlying collision system.
    /// By default, a ChSystem uses a Bullet-based collision detection engine.
    virtual void SetCollisionSystem(std::shared_ptr<collision::ChCollisionSystem> coll_sys);

    /// Access the underlying collision system.
    /// Usually this is not needed, as the collision system is automatically handled by the ChSystem.
    std::shared_ptr<collision::ChCollisionSystem> GetCollisionSystem() const { return collision_system; }

    /// Change the underlying contact container given the specified type of the collision detection system.
    /// Usually this is not needed, as the contact container is automatically handled by the ChSystem.
    /// The default implementation is a no-op, since the default contact container for a ChSystem is suitable for all
    /// types of supported collision detection systems.
    virtual void SetContactContainer(collision::ChCollisionSystemType type) {}

    /// Change the underlying contact container.
    /// The contact container collects information from the underlying collision detection system required for contact
    /// force generation. Usually this is not needed, as the contact container is automatically handled by the ChSystem.
    /// Make sure the provided contact container is compatible with both the collision detection system and the contact
    /// force formulation (NSC or SMC).
    virtual void SetContactContainer(std::shared_ptr<ChContactContainer> contactcontainer);

    /// Access the underlying contact container.
    /// Usually this is not needed, as the contact container is automatically handled by the ChSystem.
    std::shared_ptr<ChContactContainer> GetContactContainer() const { return contact_container; }

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

    /// Advances the dynamical simulation for a single step, of length step_size.
    /// This function is typically called many times in a loop in order to simulate up to a desired end time.
    int DoStepDynamics(double step_size);

    /// Performs integration until the m_endtime is exactly
    /// reached, but current time step may be automatically "retouched" to
    /// meet exactly the m_endtime after n steps.
    /// Useful when you want to advance the simulation in a
    /// simulations (3d modeling software etc.) which needs updates
    /// of the screen at a fixed rate (ex.30th of second)  while
    /// the integration must use more steps.
    bool DoFrameDynamics(double end_time);

    /// Given the current state, the sw simulates the
    /// dynamical behavior of the system, until the end
    /// time is reached, repeating many steps (maybe the step size
    /// will be automatically changed if the integrator method supports
    /// step size adaption).
    bool DoEntireDynamics(double end_time);

    /// Like "DoEntireDynamics", but results are provided at uniform
    /// steps "frame_step", using the DoFrameDynamics() many times.
    bool DoEntireUniformDynamics(double end_time, double frame_step);

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
    /// <pre>
    ///  | H -Cq'|*|q|- | f|= |0|
    ///  | Cq -E | |l|  |-b|  |c|
    /// </pre>
    /// where l \f$\in Y, c \in Ny\f$, normal cone to Y

    /// Enable/disable debug output of system matrices.
    void EnableSolverMatrixWrite(bool val, const std::string& out_dir = ".");
    bool IsSolverMatrixWriteEnabled() const { return write_matrix; }

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

    /// Advances the kinematic simulation for a single step of given length.
    bool DoStepKinematics(double step_size);

    /// Performs kinematics until the end time is exactly reached.
    /// The current time step may be automatically adjusted to meet exactly the m_endtime after n steps.
    bool DoFrameKinematics(double end_time);

    /// Given the current state, this kinematic simulation satisfies all the constraints with the "DoStepKinematics"
    /// procedure for each time step, from the current time to the end time.
    bool DoEntireKinematics(double end_time);

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

    /// Perform a generic static analysis. Low level API, where the user creates and configures a
    /// ChStaticAnalysis-inherited object by his own. For ready-to-use analysis, use 
    /// DoStaticLinear, DoStaticNonLinear, DoStaticNonlinearRheonomic etc. instead.
    bool DoStaticAnalysis(std::shared_ptr<ChStaticAnalysis> analysis);

    /// Solve the position of static equilibrium (and the reactions).
    /// This is a one-step only approach that solves the **linear** equilibrium.
    /// Appropriate mostly for FEM problems with small deformations.
    bool DoStaticLinear();

    /// Solve the position of static equilibrium (and the reactions).
    /// This function solves the equilibrium for the nonlinear problem (large displacements).
    /// This version uses a nonlinear static analysis solver with default parameters.
    bool DoStaticNonlinear(int nsteps = 10, bool verbose = false);

    /// Solve the position of static equilibrium (and the reactions).
    /// This function solves the equilibrium for the nonlinear problem (large displacements),
    /// but differently from DoStaticNonlinear, it considers rheonomic constraints (ex. ChLinkMotorRotationSpeed) 
    /// that can impose steady-state speeds&accelerations to the mechanism, ex. to generate centrifugal forces in turbine blades.
    /// This version uses a nonlinear static analysis solver with default parameters.
    bool DoStaticNonlinearRheonomic(int nsteps = 10, bool verbose = false, std::shared_ptr<ChStaticNonLinearRheonomicAnalysis::IterationCallback> mcallback = nullptr);

    /// Finds the position of static equilibrium (and the reactions) starting from the current position.
    /// Since a truncated iterative method is used, you may need to call this method multiple times in case of large
    /// nonlinearities before coming to the precise static solution.
    bool DoStaticRelaxing(int nsteps = 10);

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);

    /// Process a ".chr" binary file containing the full system object
    /// hierarchy as exported -for example- by the R3D modeler, with chrono plug-in version,
    /// or by using the FileWriteChR() function.
    int FileProcessChR(ChStreamInBinary& m_file);

    /// Write a ".chr" binary file containing the full system object
    /// hierarchy (bodies, forces, links, etc.) (deprecated function - obsolete)
    int FileWriteChR(ChStreamOutBinary& m_file);

  protected:
    ChAssembly assembly;

    std::shared_ptr<ChContactContainer> contact_container;  ///< the container of contacts

    ChVector<> G_acc;  ///< gravitational acceleration

    bool is_initialized;  ///< if false, an initial setup is required (i.e. a call to SetupInitial)
    bool is_updated;      ///< if false, a new update is required (i.e. a call to Update)

    int ncoords;     ///< number of scalar coordinates (including 4th dimension of quaternions) for all active bodies
    int ndoc;        ///< number of scalar constraints (including constr. on quaternions)
    int nsysvars;    ///< number of variables (coords+lagrangian mult.), i.e. = ncoords+ndoc  for all active bodies
    int ncoords_w;   ///< number of scalar coordinates when using 3 rot. dof. per body;  for all active bodies
    int ndoc_w;      ///< number of scalar constraints  when using 3 rot. dof. per body;  for all active bodies
    int nsysvars_w;  ///< number of variables when using 3 rot. dof. per body; i.e. = ncoords_w+ndoc_w
    int ndof;        ///< number of degrees of freedom, = ncoords-ndoc =  ncoords_w-ndoc_w ,
    int ndoc_w_C;    ///< number of scalar constraints C, when using 3 rot. dof. per body (excluding unilaterals)
    int ndoc_w_D;    ///< number of scalar constraints D, when using 3 rot. dof. per body (only unilaterals)

    double ch_time;   ///< simulation time of the system
    double step;      ///< time step
    double step_min;  ///< min time step
    double step_max;  ///< max time step

    double tol_force;  ///< tolerance for forces (used to obtain a tolerance for impulses)

    int maxiter;  ///< max iterations for nonlinear convergence in DoAssembly()

    bool use_sleeping;  ///< if true, put to sleep objects that come to rest

    std::shared_ptr<ChSystemDescriptor> descriptor;  ///< system descriptor
    std::shared_ptr<ChSolver> solver;                ///< solver for DVI or DAE problem

    double min_bounce_speed;                ///< minimum speed for rebounce after impacts. Lower speeds are clamped to 0
    double max_penetration_recovery_speed;  ///< limit for the speed of penetration recovery (positive, speed of exiting)

    size_t stepcount;  ///< internal counter for steps

    int setupcount;  ///< number of calls to the solver's Setup()
    int solvecount;  ///< number of StateSolveCorrection (reset to 0 at each timestep of static analysis)

    bool write_matrix;       ///< write current system matrix to file(s); for debugging
    std::string output_dir;  ///< output directory for writing system matrices

    int ncontacts;  ///< total number of contacts

    collision::ChCollisionSystemType collision_system_type;                     ///< type of the collision engine
    std::shared_ptr<collision::ChCollisionSystem> collision_system;             ///< collision engine
    std::vector<std::shared_ptr<CustomCollisionCallback>> collision_callbacks;  ///< user-defined collision callbacks
    std::unique_ptr<ChMaterialCompositionStrategy> composition_strategy;        /// material composition strategy

    ChVisualSystem* visual_system;  ///< run-time visualization engine

    // OpenMP
    int nthreads_chrono;
    int nthreads_eigen;
    int nthreads_collision;

    // timers for profiling execution speed
    ChTimer<double> timer_step;       ///< timer for integration step
    ChTimer<double> timer_advance;    ///< timer for time integration
    ChTimer<double> timer_ls_solve;   ///< timer for solver (excluding setup phase)
    ChTimer<double> timer_ls_setup;   ///< timer for solver setup
    ChTimer<double> timer_jacobian;   ///< timer for computing/loading Jacobian information
    ChTimer<double> timer_collision;  ///< timer for collision detection
    ChTimer<double> timer_setup;      ///< timer for system setup
    ChTimer<double> timer_update;     ///< timer for system update

    std::shared_ptr<ChTimestepper> timestepper;  ///< time-stepper object

    bool last_err;  ///< indicates error over the last kinematic/dynamics/statics

    ChVectorDynamic<> applied_forces;  ///< system-wide vector of applied forces (lazy evaluation)
    bool applied_forces_current;       ///< indicates if system-wide vector of forces is up-to-date

    // Friend class declarations

    friend class ChAssembly;
    friend class ChBody;
    friend class fea::ChMesh;

    friend class ChContactContainerNSC;
    friend class ChContactContainerSMC;

    friend class ChVisualSystem;

    friend class modal::ChModalAssembly;
};

CH_CLASS_VERSION(ChSystem, 0)

}  // end namespace chrono

#endif
