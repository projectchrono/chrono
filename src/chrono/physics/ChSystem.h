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

#include "chrono/core/ChDataPath.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChTimer.h"
#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/utils/ChOpenMP.h"
#include "chrono/physics/ChAssembly.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChSolver.h"
#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/timestepper/ChAssemblyAnalysis.h"
#include "chrono/timestepper/ChIntegrable.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/timestepper/ChTimestepperExplicit.h"
#include "chrono/timestepper/ChTimestepperImplicit.h"
#include "chrono/timestepper/ChTimestepperHHT.h"
#include "chrono/timestepper/ChStaticAnalysis.h"
#include "chrono/input_output/ChOutput.h"

namespace chrono {

// Forward references
class ChVisualSystem;

namespace modal {
class ChModalAssembly;
}

/// Chrono simulation system.
///
/// This class is the main simulation object used to collect:
/// - the simulation model itself, through the ChAssembly object;
/// - the solver and the timestepper;
/// - the collision system;
/// - system-wise parameters (time, gravitational acceleration, etc.)
/// - counters and timers;
/// as well as other secondary features.
///
/// This class is in charge of managing dynamic, kinematic, and static simulations.
///
/// Consult the @ref simulation_system manual page for additional details.
///
class ChApi ChSystem : public ChIntegrableIIorder {
  public:
    /// Create a physical system.
    ChSystem(const std::string& name = "");

    /// Copy constructor.
    ChSystem(const ChSystem& other);

    /// Destructor
    virtual ~ChSystem();

    /// "Virtual" copy constructor.
    /// Concrete derived classes must implement this.
    virtual ChSystem* Clone() const = 0;

    static std::shared_ptr<ChSystem> Create(ChContactMethod contact_method);

    /// Set the system name.
    void SetName(const std::string& name) { m_name = name; }

    /// Get the system name.
    const std::string& GetName() const { return m_name; }

    /// Set the method for time integration (time stepper type).
    ///   - Suggested for fast dynamics with hard (NSC) contacts: EULER_IMPLICIT_LINEARIZED
    ///   - Suggested for fast dynamics with hard (NSC) contacts and low inter-penetration: EULER_IMPLICIT_PROJECTED
    ///   - Suggested for finite element smooth dynamics: HHT, EULER_IMPLICIT_LINEARIZED
    /// For full access to a time stepper's settings, use SetTimestepper()
    void SetTimestepperType(ChTimestepper::Type type);

    /// Get the current method for time integration (time stepper type).
    ChTimestepper::Type GetTimestepperType() const { return timestepper->GetType(); }

    /// Set the timestepper object to be used for time integration.
    void SetTimestepper(std::shared_ptr<ChTimestepper> stepper) { timestepper = stepper; }

    /// Get the timestepper currently used for time integration
    std::shared_ptr<ChTimestepper> GetTimestepper() const { return timestepper; }

    /// Set the collision detection system used by this Chrono system to the specified type.
    virtual void SetCollisionSystemType(ChCollisionSystem::Type type);

    /// Set the collision detection system used by this Chrono system.
    virtual void SetCollisionSystem(std::shared_ptr<ChCollisionSystem> coll_system);

    /// Access the underlying collision system.
    /// Usually this is not needed, as the collision system is automatically handled by the ChSystem.
    std::shared_ptr<ChCollisionSystem> GetCollisionSystem() const { return collision_system; }

    /// Change the default composition laws for contact surface materials
    /// (coefficient of friction, cohesion, compliance, etc.)
    virtual void SetMaterialCompositionStrategy(std::unique_ptr<ChContactMaterialCompositionStrategy>&& strategy);

    /// Accessor for the current composition laws for contact surface material.
    const ChContactMaterialCompositionStrategy& GetMaterialCompositionStrategy() const { return *composition_strategy; }

    /// Set the speed limit of exiting from penetration situations (default: 0.6).
    /// Usually set a positive value, (about 0.1...2 m/s, as exiting speed).
    /// Used for unilateral constraints with the EULER_IMPLICIT_LINEARIZED time stepper.
    void SetMaxPenetrationRecoverySpeed(double value) { max_penetration_recovery_speed = value; }

    /// Attach a solver (derived from ChSolver) for use by this system.
    ///   - Suggested solver for speed, but lower precision: PSOR
    ///   - Suggested solver for higher precision: BARZILAIBORWEIN or APGD
    ///   - For problems that involve a stiffness matrix: GMRES, MINRES
    virtual void SetSolver(std::shared_ptr<ChSolver> newsolver);

    /// Access the solver currently associated with this system.
    virtual std::shared_ptr<ChSolver> GetSolver() { return solver; }

    /// Choose the solver type, to be used for the simultaneous solution of the constraints
    /// in dynamical simulations (as well as in kinematics, statics, etc.)
    /// Notes:
    ///   - This function is a shortcut, internally equivalent to a call to SetSolver().
    ///   - Only a subset of available Chrono solvers can be set through this mechanism.
    ///   - Prefer explicitly creating a solver, setting solver parameters, and then attaching the solver with
    ///     SetSolver.
    ///
    /// \deprecated This function does not support all available Chrono solvers. Prefer using SetSolver.
    void SetSolverType(ChSolver::Type type);

    /// Gets the current solver type.
    ChSolver::Type GetSolverType() const { return solver->GetType(); }

    /// Instead of using the default 'system descriptor', you can create your own custom descriptor
    /// (inherited from ChSystemDescriptor) and plug it into the system using this function.
    void SetSystemDescriptor(std::shared_ptr<ChSystemDescriptor> newdescriptor);

    /// Access directly the 'system descriptor'.
    std::shared_ptr<ChSystemDescriptor> GetSystemDescriptor() { return descriptor; }

    /// Set the gravitational acceleration vector.
    void SetGravitationalAcceleration(const ChVector3d& gacc) { G_acc = gacc; }

    /// Get the gravitatoinal acceleration vector.
    const ChVector3d& GetGravitationalAcceleration() const { return G_acc; }

    /// Get the simulation time of this system.
    double GetChTime() const { return ch_time; }

    /// Set (overwrite) the simulation time of this system.
    void SetChTime(double time) { ch_time = time; }

  public:
    /// Gets the current time step used for integration (dynamic and kinematic simulation).
    /// The value is set automatically when a dynamic or kinematic simulation is run.
    double GetStep() const { return step; }

    /// Return the total number of time steps taken so far.
    size_t GetNumSteps() const { return stepcount; }

    /// Reset to 0 the total number of time steps.
    void ResetNumSteps() { stepcount = 0; }

    // ---- DYNAMICS

    /// Advance the dynamics simulation by a single time step of given length.
    /// This function is typically called many times in a loop in order to simulate up to a desired end time.
    int DoStepDynamics(double step_size);

    /// Advance the dynamics simulation until the specified frame end time is reached.
    /// Integration proceeds with the specified time step size which may be adjusted to exactly reach the frame time.
    bool DoFrameDynamics(double frame_time, double step_size);

    // ---- KINEMATICS

    /// Advance the kinematics simulation for a single step of given length.
    AssemblyAnalysis::ExitFlag DoStepKinematics(double step_size);

    /// Advance the kinematics simulation to the specified frame end time.
    /// A system assembly analysis (inverse kinematics) is performed at step_size intervals. The last step may be
    /// adjusted to exactly reach the frame end time.
    AssemblyAnalysis::ExitFlag DoFrameKinematics(double frame_time, double step_size);

    // ---- STATICS

    /// Perform a generic static analysis.
    bool DoStaticAnalysis(ChStaticAnalysis& analysis);

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
    /// but differently from DoStaticNonlinear it considers rheonomic constraints (ex. ChLinkMotorRotationSpeed)
    /// that can impose steady-state speeds&accelerations to the mechanism, ex. to generate centrifugal forces in
    /// turbine blades. This version uses a nonlinear static analysis solver with default parameters.
    bool DoStaticNonlinearRheonomic(
        int max_num_iterations = 10,
        bool verbose = false,
        std::shared_ptr<ChStaticNonLinearRheonomicAnalysis::IterationCallback> callback = nullptr);

    /// Find the static equilibrium configuration (and the reactions) starting from the current position.
    /// Since a truncated iterative method is used, you may need to call this method multiple times in case of large
    /// nonlinearities before coming to the precise static solution.
    bool DoStaticRelaxing(double step_size, int num_iterations = 10);

    /// Return the number of calls to the solver's Solve() function.
    /// This counter is reset at each timestep.
    unsigned int GetSolverSolveCount() const { return solvecount; }

    /// Return the number of calls to the solver's Setup() function.
    /// This counter is reset at each timestep.
    unsigned int GetSolverSetupCount() const { return setupcount; }

    // ---- SYSTEM ASSEMBLY

    /// Assemble the system.
    /// The assembling is performed by satisfying constraints at position, velocity, and acceleration levels.
    /// Position-level assembling requires Newton-Raphson iterations.
    /// Velocity-level assembling is performed by taking a small integration step.
    /// Acceleration-level assembling is obtained through finite differentation.
    /// Argument 'action' can be one of AssemblyLevel enum values (POSITION, VELOCITY, ACCELERATION, or FULL).
    /// These values can also be combined using bit operations.
    /// Returns true if the assembling converged, false otherwise (impossible assembly?)
    /// The maximum number of iterations and the tolerance refer to the Newton-Raphson iteration for position-level.
    /// Iterations and tolerance for the inner linear solver must be set on the solver itself.
    AssemblyAnalysis::ExitFlag DoAssembly(int action,
                                          int max_num_iterationsNR = 6,
                                          double abstol_residualNR = 1e-10,
                                          double reltol_updateNR = 1e-6,
                                          double abstol_updateNR = 1e-6);

    /// Remove redundant constraints through QR decomposition of the constraints Jacobian matrix.
    /// This function can be used to improve the stability and performance of the system by removing redundant
    /// constraints. The function returns the number of redundant constraints that were deactivated/removed. Please
    /// consider that some constraints might falsely appear as redundant in a specific configuration, while they
    /// might be not in general.
    unsigned int RemoveRedundantConstraints(
        bool remove_links = false,  ///< false: redundant links are just deactivated; true: redundant links get removed
        double qr_tol = 1e-6,       ///< tolerance in QR decomposition to identify linearly dependent constraints
        bool verbose = false        ///< set verbose output
    );

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
    ///   num_threads_chrono = 1
    ///   num_threads_collision = 1
    ///   num_threads_eigen = 1
    /// </pre>
    /// Note that a derived class may ignore some or all of these settings.
    virtual void SetNumThreads(int num_threads_chrono, int num_threads_collision = 0, int num_threads_eigen = 0);

    unsigned int GetNumThreadsChrono() const { return nthreads_chrono; }
    unsigned int GetNumThreadsCollision() const { return nthreads_collision; }
    unsigned int GetNumThreadsEigen() const { return nthreads_eigen; }

    // DATABASE HANDLING

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
    virtual void RemoveBody(std::shared_ptr<ChBody> body);

    /// Remove a shaft from this assembly.
    virtual void RemoveShaft(std::shared_ptr<ChShaft> shaft);

    /// Remove a link from this assembly.
    virtual void RemoveLink(std::shared_ptr<ChLinkBase> link);

    /// Remove a mesh from the assembly.
    virtual void RemoveMesh(std::shared_ptr<fea::ChMesh> mesh);

    /// Remove a ChPhysicsItem object that is not a body or a link
    virtual void RemoveOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item);

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
    const std::vector<std::shared_ptr<ChBody>>& GetBodies() const { return assembly.bodylist; }
    /// Get the list of shafts.
    const std::vector<std::shared_ptr<ChShaft>>& GetShafts() const { return assembly.shaftlist; }
    /// Get the list of links.
    const std::vector<std::shared_ptr<ChLinkBase>>& GetLinks() const { return assembly.linklist; }
    /// Get the list of meshes.
    const std::vector<std::shared_ptr<fea::ChMesh>>& GetMeshes() const { return assembly.meshlist; }
    /// Get the list of physics items that are not in the body or link lists.
    const std::vector<std::shared_ptr<ChPhysicsItem>>& GetOtherPhysicsItems() const {
        return assembly.otherphysicslist;
    }

    /// Search a body by its name.
    std::shared_ptr<ChBody> SearchBody(const std::string& name) const { return assembly.SearchBody(name); }
    /// Search a body by its ID
    std::shared_ptr<ChBody> SearchBodyID(int id) const { return assembly.SearchBodyID(id); }
    /// Search a shaft by its name.
    std::shared_ptr<ChShaft> SearchShaft(const std::string& name) const { return assembly.SearchShaft(name); }
    /// Search a link by its name.
    std::shared_ptr<ChLinkBase> SearchLink(const std::string& name) const { return assembly.SearchLink(name); }
    /// Search a mesh by its name.
    std::shared_ptr<fea::ChMesh> SearchMesh(const std::string& name) const { return assembly.SearchMesh(name); }
    /// Search from other ChPhysics items (not bodies, links, or meshes) by name.
    std::shared_ptr<ChPhysicsItem> SearchOtherPhysicsItem(const std::string& name) const {
        return assembly.SearchOtherPhysicsItem(name);
    }
    /// Search a marker by its name.
    std::shared_ptr<ChMarker> SearchMarker(const std::string& name) const { return assembly.SearchMarker(name); }
    /// Search a marker by its unique ID.
    std::shared_ptr<ChMarker> SearchMarker(int id) const { return assembly.SearchMarker(id); }
    /// Search an item (body, link or other ChPhysics items) by name.
    std::shared_ptr<ChPhysicsItem> Search(const std::string& name) const { return assembly.Search(name); }

    /// Get the total number of bodies added to the system, including fixed and sleeping bodies.
    virtual unsigned int GetNumBodies() const { return assembly.GetNumBodies(); }

    /// Get the number of active bodies, excluding sleeping or fixed.
    virtual unsigned int GetNumBodiesActive() const { return assembly.GetNumBodiesActive(); }

    /// Get the number of sleeping bodies.
    virtual unsigned int GetNumBodiesSleeping() const { return assembly.GetNumBodiesSleeping(); }

    /// Get the number of bodies fixed to ground.
    virtual unsigned int GetNumBodiesFixed() const { return assembly.GetNumBodiesFixed(); }

    /// Get the number of shafts.
    virtual unsigned int GetNumShafts() const { return assembly.GetNumShafts(); }

    /// Get the number of shafts that are in sleeping mode (excluding fixed shafts).
    virtual unsigned int GetNumShaftsSleeping() const { return assembly.GetNumBodiesSleeping(); }

    /// Get the number of shafts that are fixed to ground.
    virtual unsigned int GetNumShaftsFixed() const { return assembly.GetNumShaftsFixed(); }

    /// Get the total number of shafts added to the assembly, including the grounded and sleeping shafts.
    virtual unsigned int GetNumShaftsTotal() const { return assembly.GetNumShaftsTotal(); }

    /// Get the number of links (including non active).
    virtual unsigned int GetNumLinks() const { return assembly.GetNumLinks(); }

    /// Get the number of active links.
    virtual unsigned int GetNumLinksActive() const { return assembly.GetNumLinksActive(); }

    /// Get the number of meshes.
    virtual unsigned int GetNumMeshes() const { return assembly.GetNumMeshes(); }

    /// Get the number of other physics items (including non active).
    virtual unsigned int GetNumOtherPhysicsItems() const { return assembly.GetNumOtherPhysicsItems(); }

    /// Get the number of other active physics items.
    virtual unsigned int GetNumOtherPhysicsItemsActive() const { return assembly.GetNumOtherPhysicsItemsActive(); }

    /// Write the hierarchy of contained bodies, markers, etc. in ASCII
    /// readable form, mostly for debugging purposes. Level is the tab spacing at the left.
    void ShowHierarchy(std::ostream& m_file, int level = 0) const { assembly.ShowHierarchy(m_file, level); }

    /// Removes all bodies/marker/forces/links/contacts, also resets timers and events.
    void Clear();

    /// Return the contact method supported by this system.
    /// Contactables (bodies, FEA nodes, FEA traiangles, etc.) added to this system must be compatible.
    virtual ChContactMethod GetContactMethod() const = 0;

    // UTILITY FUNCTIONS

    /// Executes custom processing at the end of step. By default it does nothing,
    /// but if you inherit a special ChSystem you can implement this.
    virtual void CustomEndOfStep() {}

    /// Perform the collision detection, returning the number of contacts.
    /// New contacts are inserted in the ChContactContainer object(s), and old ones are removed.
    /// This is mostly called automatically by time integration.
    unsigned int ComputeCollisions();

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

    /// Change the underlying contact container.
    /// The contact container collects information from the underlying collision detection system required for contact
    /// force generation. Usually this is not needed, as the contact container is automatically handled by the ChSystem.
    /// Make sure the provided contact container is compatible with both the collision detection system and the contact
    /// force formulation (NSC or SMC).
    virtual void SetContactContainer(std::shared_ptr<ChContactContainer> container);

    /// Access the underlying contact container.
    /// Usually this is not needed, as the contact container is automatically handled by the ChSystem.
    std::shared_ptr<ChContactContainer> GetContactContainer() const { return contact_container; }

    /// Turn on this feature to let the system put to sleep the bodies whose
    /// motion has almost come to a rest. This feature will allow faster simulation
    /// of large scenarios for real-time purposes, but it will affect the precision!
    /// This functionality can be turned off selectively for specific ChBodies.
    void SetSleepingAllowed(bool ms) { use_sleeping = ms; }

    /// Tell if the system will put to sleep the bodies whose motion has almost come to a rest.
    bool IsSleepingAllowed() const { return use_sleeping; }

    /// Get the visual system to which this ChSystem is attached (if any).
    ChVisualSystem* GetVisualSystem() const { return visual_system; }

    // STATISTICS

    /// Gets the number of contacts.
    virtual unsigned int GetNumContacts();

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
    double GetTimerCollisionBroad() const;

    /// Return the time (in seconds) for narrowphase collision detection, within the time step.
    double GetTimerCollisionNarrow() const;

    /// Get current estimated RTF (real time factor).
    /// This represents the real time factor for advancing the dynamic state of the system only and as such does not
    /// take into account any other operations performed during a step (e.g., run-time visualization). During each call
    /// to DoStepDynamics(), this value is calculated as T/step_size, where T includes the time spent in system setup,
    /// collision detection, and integration.
    double GetRTF() const { return m_RTF; }

    /// Set (overwrite) the RTF value for this system (if calculated externally).
    void SetRTF(double rtf) { m_RTF = rtf; }

    /// Resets the timers.
    void ResetTimers();

    /// DEBUGGING

    /// Enable/disable debug output of system matrices.
    /// Set this to "true" to enable automatic saving of solver matrices at each time step, for debugging purposes.
    /// Matrices will be saved in 'out_dir' (default to the the working directory of the executable).
    /// The name pattern is solve_numstep_numsubstep_objectname.dat
    /// where 'objectname' can be either:
    /// - Z: the assembled optimization matrix [H, -Cq'; Cq, -E]
    /// - rhs: the assembled right-hand side vector [f; -b]
    /// - x_pre: the position vector before the integration step
    /// - v_pre: the velocity vector before the integration step
    /// - F_pre: unscaled loads before the integration step
    /// - Dv: the state variation
    /// - Dl: the Lagrange multipliers variation
    /// if the solver is direct then also the following are output:
    /// - f, b: subparts of 'rhs'
    /// - H, Cq, E: the submatrices of Z
    /// as passed to the solver in the problem
    /// <pre>
    /// | H  Cq'|*| q|-| f|=|0|
    /// | Cq  E | |-l| |-b| |c|
    /// </pre>
    /// where l \f$\in Y, c \in Ny\f$, normal cone to Y
    void EnableSolverMatrixWrite(bool val, const std::string& out_dir = ".");

    /// Return true if solver matrices are being written to disk.
    bool IsSolverMatrixWriteEnabled() const { return write_matrix; }

    /// Write the mass (M), damping (R), stiffness (K), and constraint Jacobian (Cq) matrices at current configuration.
    /// These can be used for linearized motion, modal analysis, buckling analysis, etc.
    /// The sparse matrices are saved in COO format in 'path' folder according to the naming:
    /// solve_M.dat solve_K.dat solve_R.dat, and solve_Cq.dat.
    /// By default, uses 1-based indices (as in Matlab).
    void WriteSystemMatrices(bool save_M,
                             bool save_K,
                             bool save_R,
                             bool save_Cq,
                             const std::string& path,
                             bool one_indexed = true);

    /// Compute the system-level mass matrix and load in the provided sparse matrix.
    void GetMassMatrix(ChSparseMatrix& M);

    /// Compute the system-level stiffness matrix and load in the provided sparse matrix.
    /// This is the Jacobian -dF/dq, where F are stiff loads.
    /// Note that not all loads provide a jacobian, as this is optional in their implementation.
    void GetStiffnessMatrix(ChSparseMatrix& K);

    /// Compute the system-level damping matrix and load in the provided sparse matrix.
    /// This is the Jacobian -dF/dv, where F are stiff loads.
    /// Note that not all loads provide a Jacobian, as this is optional in their implementation.
    void GetDampingMatrix(ChSparseMatrix& R);

    /// Compute the system-level constraint Jacobian matrix and load in the provided sparse matrix.
    /// This is the Jacobian Cq=-dC/dq, where C are constraints (the lower left part of the KKT matrix).
    void GetConstraintJacobianMatrix(ChSparseMatrix& Cq);

    // ---- OUTPUT

    void Output(int frame, ChOutput& database) const;

    // ---- SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

  public:
    /// Counts the number of bodies and links.
    /// Computes the offsets of object states in the global state. Assumes that offset_x, offset_w, and offset_L are
    /// already set as starting point for offsetting all the contained sub objects.
    virtual void Setup();

    /// Updates all the auxiliary data and children of bodies, forces, links, given their current state.
    void Update(double time, bool update_assets);

    /// Updates all the auxiliary data and children of bodies, forces, links, given their current state.
    void Update(bool update_assets);

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

    /// Register with the given system descriptor all ChVariable objects associated with items in the system.
    void InjectVariables(ChSystemDescriptor& sys_descriptor);

    /// Register with the given system descriptor any ChConstraint objects associated with items in the system.
    void InjectConstraints(ChSystemDescriptor& sys_descriptor);

    /// Compute and load current Jacobians in encapsulated ChConstraint objects.
    void LoadConstraintJacobians();

    /// Register with the given system descriptor any ChKRMBlock objects associated with items in the system.
    void InjectKRMMatrices(ChSystemDescriptor& sys_descriptor);

    /// Compute and load current stiffnes (K), damping (R), and mass (M) matrices in encapsulated ChKRMBlock objects.
    /// The resulting KRM blocks represent linear combinations of the K, R, and M matrices, with the specified
    /// coefficients Kfactor, Rfactor,and Mfactor, respectively.
    void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor);

    // Old bookkeeping system
    void VariablesFbReset();
    void VariablesFbLoadForces(double factor = 1);
    void VariablesQbLoadSpeed();
    void VariablesFbIncrementMq();
    void VariablesQbSetSpeed(double step_size = 0);
    void VariablesQbIncrementPosition(double step_size);
    void ConstraintsBiReset();
    void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false);
    void ConstraintsBiLoad_Ct(double factor = 1);
    void ConstraintsBiLoad_Qc(double factor = 1);
    void ConstraintsFbLoadForces(double factor = 1);
    void ConstraintsFetch_react(double factor = 1);

    // TIMESTEPPER INTERFACE

    /// Get the number of coordinates at the position level.
    /// Might differ from coordinates at the velocity level if quaternions are used for rotations.
    virtual unsigned int GetNumCoordsPosLevel() override { return m_num_coords_pos; }

    /// Get the number of coordinates at the velocity level.
    /// Might differ from coordinates at the position level if quaternions are used for rotations.
    virtual unsigned int GetNumCoordsVelLevel() override { return m_num_coords_vel; }

    /// Get the number of scalar constraints in the system.
    virtual unsigned int GetNumConstraints() override { return m_num_constr; }

    /// Get the number of bilateral scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() { return m_num_constr_bil; }

    /// Get the number of unilateral scalar constraints.
    virtual unsigned int GetNumConstraintsUnilateral() { return m_num_constr_uni; }

    /// From system to state y={x,v}
    virtual void StateGather(ChState& x, ChStateDelta& v, double& T) override;

    /// From state Y={x,v} to system. This also triggers an update operation.
    virtual void StateScatter(const ChState& x, const ChStateDelta& v, const double T, bool full_update) override;

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

    /// Return true if the number of states or Jacobian structure has changed.
    /// In such cases, an implicit integrator should force a Jacobian re-evaluation.
    /// For a ChSystem, this happens when a physics item is added to or removed from the underlying assembly.
    virtual bool StateModified() const override { return !is_updated; }

    /// Assuming a DAE of the form
    /// <pre>
    ///       M*a = F(x,v,t) + Cq'*L
    ///       C(x,t) = 0
    /// </pre>
    /// this function computes the solution of the change Du (in a or v or x) for a Newton
    /// iteration within an implicit integration scheme.
    /// <pre>
    ///  | Du| = [ G   Cq' ]^-1 * | R |
    ///  |-DL|   [ Cq  0   ]      |-Qc|
    /// </pre>
    /// for residual R and  G = [ c_a*M + c_v*dF/dv + c_x*dF/dx ].\n
    /// This function returns true if successful and false otherwise.
    virtual bool StateSolveCorrection(
        ChStateDelta& Dv,             ///< result: computed Dv
        ChVectorDynamic<>& DL,        ///< result: computed Lagrange multipliers
        const ChVectorDynamic<>& R,   ///< the R residual
        const ChVectorDynamic<>& Qc,  ///< the Qc residual
        const double c_a,             ///< the factor in c_a*M
        const double c_v,             ///< the factor in c_v*dF/dv
        const double c_x,             ///< the factor in c_x*dF/dv
        const ChState& x,             ///< current state, x part
        const ChStateDelta& v,        ///< current state, v part
        const double T,               ///< current time T
        bool force_state_scatter,     ///< if true, scatter x and v to the system
        bool full_update,             ///< if true, perform a full update during scatter
        bool call_setup,              ///< if true, call the solver's Setup function
        bool call_analyze             ///< if true, call the solver's Setup analyze phase
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

    /// Adds the lumped mass to a Md vector, representing a mass diagonal matrix. Used by lumped explicit integrators.
    /// If mass lumping is impossible or approximate, adds scalar error to "error" parameter.
    ///    Md += c*diag(M)    or   Md += c*HRZ(M)
    virtual void LoadLumpedMass_Md(ChVectorDynamic<>& Md,  ///< result: Md vector, diagonal of the lumped mass matrix
                                   double& err,            ///< result: not touched if lumping does not introduce errors
                                   const double c          ///< a scaling factor
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

  protected:
    /// Pushes all ChConstraints and ChVariables contained in links, bodies, etc. into the system descriptor.
    virtual void DescriptorPrepareInject(ChSystemDescriptor& sys_descriptor);

    /// Initial system setup before analysis.
    /// This function performs an initial system setup, once system construction is completed and before an analysis.
    /// This function also initializes the collision system (if any), as well as any visualization system to which this
    /// Chrono system was attached. The initialization function is called automatically before starting any type of
    /// analysis.
    void Initialize();

    /// Return the resultant applied force on the specified body.
    /// This resultant force includes all external applied loads acting on the body (from gravity, loads, springs,
    /// etc). However, this does *not* include any constraint forces. In particular, contact forces are not included if
    /// using the NSC formulation, but are included when using the SMC formulation.
    virtual ChVector3d GetBodyAppliedForce(ChBody* body);

    /// Return the resultant applied torque on the specified body.
    /// This resultant torque includes all external applied loads acting on the body (from gravity, loads, springs,
    /// etc). However, this does *not* include any constraint forces. In particular, contact torques are not included if
    /// using the NSC formulation, but are included when using the SMC formulation.
    virtual ChVector3d GetBodyAppliedTorque(ChBody* body);

    /// Put bodies to sleep if possible. Also awakens sleeping bodies, if needed.
    /// Returns true if some body changed from sleep to no sleep or viceversa,
    /// returns false if nothing changed. In the former case also performs Setup()
    /// since the system changed.
    bool ManageSleepingBodies();

    /// Performs a single dynamics simulation step, advancing the system state by the current step size.
    virtual bool AdvanceDynamics();

    std::string m_name;                                       ///< system name
    ChAssembly assembly;                                    ///< underlying mechanical assembly
    std::shared_ptr<ChContactContainer> contact_container;  ///< the container of contacts

    ChVector3d G_acc;  ///< gravitational acceleration

    bool is_initialized;  ///< if false, an initial setup is required (i.e. a call to Initialize)
    bool is_updated;      ///< if false, a new update is required (i.e. a call to Update)

    unsigned int m_num_coords_pos;  ///< num of scalar coordinates at position level for all active bodies
    unsigned int m_num_coords_vel;  ///< num of scalar coordinates at velocity level for all active bodies
    unsigned int m_num_constr;      ///< num of scalar constraints (at velocity level) for all active constraints
    unsigned int m_num_constr_bil;  ///< num of scalar bilateral active constraints (velocity level)
    unsigned int m_num_constr_uni;  ///< num of scalar unilateral active constraints (velocity level)

    double ch_time;  ///< simulation time of the system
    double step;     ///< time step

    bool use_sleeping;  ///< if true, put to sleep objects that come to rest

    std::shared_ptr<ChSystemDescriptor> descriptor;  ///< system descriptor
    std::shared_ptr<ChSolver> solver;                ///< solver for DVI or DAE problem

    double max_penetration_recovery_speed;  ///< limit for speed of penetration recovery (positive)

    size_t stepcount;  ///< internal counter for steps

    unsigned int setupcount;  ///< number of calls to the solver's Setup()
    unsigned int solvecount;  ///< number of StateSolveCorrection (reset to 0 at each timestep of static analysis)

    bool write_matrix;       ///< write current system matrix to file(s); for debugging
    std::string output_dir;  ///< output directory for writing system matrices

    unsigned int ncontacts;  ///< total number of contacts

    std::shared_ptr<ChCollisionSystem> collision_system;                         ///< collision engine
    std::vector<std::shared_ptr<CustomCollisionCallback>> collision_callbacks;   ///< user-defined collision callbacks
    std::unique_ptr<ChContactMaterialCompositionStrategy> composition_strategy;  /// material composition strategy

    ChVisualSystem* visual_system;  ///< run-time visualization engine

    // OpenMP
    int nthreads_chrono;
    int nthreads_eigen;
    int nthreads_collision;

    // timers for profiling execution speed
    ChTimer timer_step;       ///< timer for integration step
    ChTimer timer_advance;    ///< timer for time integration
    ChTimer timer_ls_solve;   ///< timer for solver (excluding setup phase)
    ChTimer timer_ls_setup;   ///< timer for solver setup
    ChTimer timer_jacobian;   ///< timer for computing/loading Jacobian information
    ChTimer timer_collision;  ///< timer for collision detection
    ChTimer timer_setup;      ///< timer for system setup
    ChTimer timer_update;     ///< timer for system update
    double m_RTF;             ///< real-time factor (simulation time / simulated time)

    std::shared_ptr<ChTimestepper> timestepper;  ///< time-stepper object

    ChVectorDynamic<> applied_forces;  ///< system-wide vector of applied forces (lazy evaluation)
    bool applied_forces_current;       ///< indicates if system-wide vector of forces is up-to-date

    // Friend class declarations

    friend class ChAssembly;
    friend class ChBody;
    friend class fea::ChMesh;

    friend class ChContactContainerNSC;
    friend class ChContactContainerSMC;

    friend class ChVisualSystem;
    friend class ChCollisionSystem;

    friend class modal::ChModalAssembly;
};

CH_CLASS_VERSION(ChSystem, 0)

}  // end namespace chrono

#endif
