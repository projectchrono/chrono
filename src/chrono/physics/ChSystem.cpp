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

#include <algorithm>

#include "chrono/collision/ChCollisionSystemBullet.h"
#include "chrono/physics/ChProximityContainer.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/solver/ChSolverAPGD.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/solver/ChSolverPJacobi.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/solver/ChSolverPSSOR.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/utils/ChProfiler.h"

using namespace chrono::collision;

namespace chrono {

// -----------------------------------------------------------------------------
// CLASS FOR PHYSICAL SYSTEM
// -----------------------------------------------------------------------------

ChSystem::ChSystem()
    : G_acc(ChVector<>(0, -9.8, 0)),
      is_initialized(false),
      is_updated(false),
      ncoords(0),
      ndoc(0),
      nsysvars(0),
      ncoords_w(0),
      ndoc_w(0),
      nsysvars_w(0),
      ndof(0),
      ndoc_w_C(0),
      ndoc_w_D(0),
      ch_time(0),
      step(0.04),
      step_min(0.002),
      step_max(0.04),
      tol_force(-1),
      maxiter(6),
      use_sleeping(false),
      min_bounce_speed(0.15),
      max_penetration_recovery_speed(0.6),
      stepcount(0),
      setupcount(0),
      solvecount(0),
      dump_matrices(false),
      ncontacts(0),
      composition_strategy(new ChMaterialCompositionStrategy),
      nthreads_chrono(ChOMP::GetNumProcs()),
      nthreads_eigen(1),
      nthreads_collision(1),
      last_err(false),
      applied_forces_current(false) {
    assembly.system = this;

    // Set default collision envelope and margin.
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.03);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.01);

    // Set default timestepper.
    timestepper = chrono_types::make_shared<ChTimestepperEulerImplicitLinearized>(this);
}

ChSystem::ChSystem(const ChSystem& other) {
    // Required by ChAssembly
    assembly = other.assembly;
    assembly.system = this;

    G_acc = other.G_acc;
    ncoords = other.ncoords;
    ncoords_w = other.ncoords_w;
    ndoc = other.ndoc;
    ndoc_w = other.ndoc_w;
    ndoc_w_C = other.ndoc_w_C;
    ndoc_w_D = other.ndoc_w_D;
    ndof = other.ndof;
    nsysvars = other.nsysvars;
    nsysvars_w = other.nsysvars_w;
    ch_time = other.ch_time;
    step = other.step;
    step_min = other.step_min;
    step_max = other.step_max;
    stepcount = other.stepcount;
    solvecount = other.solvecount;
    setupcount = other.setupcount;
    dump_matrices = other.dump_matrices;
    SetTimestepperType(other.GetTimestepperType());
    tol_force = other.tol_force;
    nthreads_chrono = other.nthreads_chrono;
    nthreads_eigen = other.nthreads_eigen;
    nthreads_collision = other.nthreads_collision;
    is_initialized = false;
    is_updated = false;
    applied_forces_current = false;
    maxiter = other.maxiter;

    min_bounce_speed = other.min_bounce_speed;
    max_penetration_recovery_speed = other.max_penetration_recovery_speed;
    SetSolverType(other.GetSolverType());
    use_sleeping = other.use_sleeping;

    ncontacts = other.ncontacts;

    collision_callbacks = other.collision_callbacks;

    last_err = other.last_err;
}

ChSystem::~ChSystem() {
    // Before proceeding, anticipate Clear(). This would be called also by base ChAssembly destructor, anyway, but
    // it would happen after this destructor, so the ith_body->SetSystem(0) in Clear() would not be able to remove
    // body collision models from the collision_system. Here it is possible, since the collision_system is still alive.
    Clear();
}

void ChSystem::Clear() {
    assembly.Clear();

    // contact_container->RemoveAllContacts();

    // ResetTimers();
}

// -----------------------------------------------------------------------------

// Add arbitrary physics item to the underlying assembly.
// NOTE: we cannot simply invoke ChAssembly::Add as this would not provide
// polymorphism!
void ChSystem::Add(std::shared_ptr<ChPhysicsItem> item) {
    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        AddBody(body);
        return;
    }

    if (auto link = std::dynamic_pointer_cast<ChLinkBase>(item)) {
        AddLink(link);
        return;
    }

    if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
        AddMesh(mesh);
        return;
    }

    AddOtherPhysicsItem(item);
}

void ChSystem::Remove(std::shared_ptr<ChPhysicsItem> item) {
    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        RemoveBody(body);
        return;
    }

    if (auto link = std::dynamic_pointer_cast<ChLinkBase>(item)) {
        RemoveLink(link);
        return;
    }

    if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
        RemoveMesh(mesh);
        return;
    }

    RemoveOtherPhysicsItem(item);
}

// -----------------------------------------------------------------------------
// Set/Get routines
// -----------------------------------------------------------------------------

void ChSystem::SetSolverMaxIterations(int max_iters) {
    if (auto iter_solver = std::dynamic_pointer_cast<ChIterativeSolver>(solver)) {
        iter_solver->SetMaxIterations(max_iters);
    }
}

int ChSystem::GetSolverMaxIterations() const {
    if (auto iter_solver = std::dynamic_pointer_cast<ChIterativeSolver>(solver)) {
        return iter_solver->GetMaxIterations();
    }
    return 0;
}

void ChSystem::SetSolverTolerance(double tolerance) {
    if (auto iter_solver = std::dynamic_pointer_cast<ChIterativeSolver>(solver)) {
        iter_solver->SetTolerance(tolerance);
    }
}

double ChSystem::GetSolverTolerance() const {
    if (auto iter_solver = std::dynamic_pointer_cast<ChIterativeSolver>(solver)) {
        return iter_solver->GetTolerance();
    }
    return 0;
}

void ChSystem::SetSolverType(ChSolver::Type type) {
    // Do nothing if changing to a CUSTOM solver.
    if (type == ChSolver::Type::CUSTOM)
        return;

    descriptor = chrono_types::make_shared<ChSystemDescriptor>();

    switch (type) {
        case ChSolver::Type::PSOR:
            solver = chrono_types::make_shared<ChSolverPSOR>();
            break;
        case ChSolver::Type::PSSOR:
            solver = chrono_types::make_shared<ChSolverPSSOR>();
            break;
        case ChSolver::Type::PJACOBI:
            solver = chrono_types::make_shared<ChSolverPJacobi>();
            break;
        case ChSolver::Type::PMINRES:
            solver = chrono_types::make_shared<ChSolverPMINRES>();
            break;
        case ChSolver::Type::BARZILAIBORWEIN:
            solver = chrono_types::make_shared<ChSolverBB>();
            break;
        case ChSolver::Type::APGD:
            solver = chrono_types::make_shared<ChSolverAPGD>();
            break;
        case ChSolver::Type::GMRES:
            solver = chrono_types::make_shared<ChSolverGMRES>();
            break;
        case ChSolver::Type::MINRES:
            solver = chrono_types::make_shared<ChSolverMINRES>();
            break;
        default:
            GetLog() << "Solver type not supported. Use SetSolver instead.\n";
            break;
    }
}

std::shared_ptr<ChSolver> ChSystem::GetSolver() {
    // In case the solver is iterative, and if the user specified a force-level tolerance,
    // overwrite the solver's tolerance threshold.
    if (auto iter_solver = std::dynamic_pointer_cast<ChIterativeSolver>(solver)) {
        if (tol_force > 0) {
            iter_solver->SetTolerance(tol_force * step);
        }
    }

    return solver;
}

// Plug-in components configuration

void ChSystem::SetSystemDescriptor(std::shared_ptr<ChSystemDescriptor> newdescriptor) {
    assert(newdescriptor);
    descriptor = newdescriptor;
}
void ChSystem::SetSolver(std::shared_ptr<ChSolver> newsolver) {
    assert(newsolver);
    solver = newsolver;
}

void ChSystem::SetContactContainer(std::shared_ptr<ChContactContainer> container) {
    assert(container);
    contact_container = container;
    contact_container->SetSystem(this);
}

void ChSystem::SetCollisionSystem(std::shared_ptr<ChCollisionSystem> newcollsystem) {
    assert(assembly.GetNbodies() == 0);
    assert(newcollsystem);
    collision_system = newcollsystem;
    collision_system->SetNumThreads(nthreads_collision);
}

void ChSystem::SetMaterialCompositionStrategy(std::unique_ptr<ChMaterialCompositionStrategy>&& strategy) {
    composition_strategy = std::move(strategy);
}

void ChSystem::SetNumThreads(int num_threads_chrono, int num_threads_collision, int num_threads_eigen) {
    nthreads_chrono = std::max(1, num_threads_chrono);
    nthreads_collision = (num_threads_collision == 0) ? num_threads_chrono : num_threads_collision;
    nthreads_eigen = (num_threads_eigen == 0) ? num_threads_chrono : num_threads_eigen;
}

// -----------------------------------------------------------------------------

// Initial system setup before analysis.
// This function must be called once the system construction is completed.
void ChSystem::SetupInitial() {
    // Set num threads for Eigen
    Eigen::setNbThreads(nthreads_eigen);

    // Set num threads for the collision system
    if (collision_system) {
        collision_system->SetNumThreads(nthreads_collision);
    }

    assembly.SetupInitial();
    is_initialized = true;
}

// -----------------------------------------------------------------------------
// HIERARCHY HANDLERS
// -----------------------------------------------------------------------------

void ChSystem::Reference_LM_byID() {
    std::vector<std::shared_ptr<ChLinkBase>> toremove;

    for (auto& link : assembly.linklist) {
        if (auto malink = std::dynamic_pointer_cast<ChLinkMarkers>(link)) {
            std::shared_ptr<ChMarker> shm1 = assembly.SearchMarker(malink->GetMarkID1());
            std::shared_ptr<ChMarker> shm2 = assembly.SearchMarker(malink->GetMarkID2());
            ChMarker* mm1 = shm1.get();
            ChMarker* mm2 = shm1.get();
            malink->SetUpMarkers(mm1, mm2);
            if (mm1 && mm2) {
                malink->SetValid(true);
            } else {
                malink->SetValid(false);
                malink->SetUpMarkers(0, 0);  // note: marker IDs are maintained
                toremove.push_back(malink);
            }
        }
    }

    for (int ir = 0; ir < toremove.size(); ++ir) {
        assembly.RemoveLink(toremove[ir]);
    }
}

// -----------------------------------------------------------------------------
// PREFERENCES
// -----------------------------------------------------------------------------

void ChSystem::SetTimestepperType(ChTimestepper::Type type) {
    // Do nothing if changing to a CUSTOM timestepper.
    if (type == ChTimestepper::Type::CUSTOM)
        return;

    // Do nothing, if no change from current typestepper.
    if (type == GetTimestepperType())
        return;

    // Plug in the new required timestepper
    // (the previous will be automatically deallocated thanks to shared pointers)
    switch (type) {
        case ChTimestepper::Type::EULER_IMPLICIT:
            timestepper = chrono_types::make_shared<ChTimestepperEulerImplicit>(this);
            std::static_pointer_cast<ChTimestepperEulerImplicit>(timestepper)->SetMaxiters(4);
            break;
        case ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED:
            timestepper = chrono_types::make_shared<ChTimestepperEulerImplicitLinearized>(this);
            break;
        case ChTimestepper::Type::EULER_IMPLICIT_PROJECTED:
            timestepper = chrono_types::make_shared<ChTimestepperEulerImplicitProjected>(this);
            break;
        case ChTimestepper::Type::TRAPEZOIDAL:
            timestepper = chrono_types::make_shared<ChTimestepperTrapezoidal>(this);
            std::static_pointer_cast<ChTimestepperTrapezoidal>(timestepper)->SetMaxiters(4);
            break;
        case ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED:
            timestepper = chrono_types::make_shared<ChTimestepperTrapezoidalLinearized>(this);
            std::static_pointer_cast<ChTimestepperTrapezoidalLinearized>(timestepper)->SetMaxiters(4);
            break;
        case ChTimestepper::Type::HHT:
            timestepper = chrono_types::make_shared<ChTimestepperHHT>(this);
            std::static_pointer_cast<ChTimestepperHHT>(timestepper)->SetMaxiters(4);
            break;
        case ChTimestepper::Type::HEUN:
            timestepper = chrono_types::make_shared<ChTimestepperHeun>(this);
            break;
        case ChTimestepper::Type::RUNGEKUTTA45:
            timestepper = chrono_types::make_shared<ChTimestepperRungeKuttaExpl>(this);
            break;
        case ChTimestepper::Type::EULER_EXPLICIT:
            timestepper = chrono_types::make_shared<ChTimestepperEulerExplIIorder>(this);
            break;
        case ChTimestepper::Type::LEAPFROG:
            timestepper = chrono_types::make_shared<ChTimestepperLeapfrog>(this);
            break;
        case ChTimestepper::Type::NEWMARK:
            timestepper = chrono_types::make_shared<ChTimestepperNewmark>(this);
            break;
        default:
            throw ChException("SetTimestepperType: timestepper not supported");
    }
}

bool ChSystem::ManageSleepingBodies() {
    if (!GetUseSleeping())
        return 0;

    // STEP 1:
    // See if some body could change from no sleep-> sleep

    for (auto& body : assembly.bodylist) {
        // mark as 'could sleep' candidate
        body->TrySleeping();
    }

    // STEP 2:
    // See if some sleeping or potential sleeping body is touching a non sleeping one,
    // if so, set to no sleep.

    // Make this class for iterating through contacts

    class _wakeup_reporter_class : public ChContactContainer::ReportContactCallback {
      public:
        // Callback, used to report contact points already added to the container.
        // If returns false, the contact scanning will be stopped.
        virtual bool OnReportContact(
            const ChVector<>& pA,             // get contact pA
            const ChVector<>& pB,             // get contact pB
            const ChMatrix33<>& plane_coord,  // get contact plane coordsystem (A column 'X' is contact normal)
            const double& distance,           // get contact distance
            const double& eff_radius,         // effective radius of curvature at contact
            const ChVector<>& react_forces,   // get react.forces (if already computed). In coordsystem 'plane_coord'
            const ChVector<>& react_torques,  // get react.torques, if rolling friction (if already computed).
            ChContactable* contactobjA,  // get model A (note: some containers may not support it and could be zero!)
            ChContactable* contactobjB   // get model B (note: some containers may not support it and could be zero!)
            ) override {
            if (!(contactobjA && contactobjB))
                return true;
            ChBody* b1 = dynamic_cast<ChBody*>(contactobjA);
            ChBody* b2 = dynamic_cast<ChBody*>(contactobjB);
            if (!(b1 && b2))
                return true;
            bool sleep1 = b1->GetSleeping();
            bool sleep2 = b2->GetSleeping();
            bool could_sleep1 = b1->BFlagGet(ChBody::BodyFlag::COULDSLEEP);
            bool could_sleep2 = b2->BFlagGet(ChBody::BodyFlag::COULDSLEEP);
            bool ground1 = b1->GetBodyFixed();
            bool ground2 = b2->GetBodyFixed();
            if (sleep1 && !(sleep2 || could_sleep2) && !ground2) {
                b1->SetSleeping(false);
                need_Setup_A = true;
            }
            if (sleep2 && !(sleep1 || could_sleep1) && !ground1) {
                b2->SetSleeping(false);
                need_Setup_A = true;
            }
            if (could_sleep1 && !(sleep2 || could_sleep2) && !ground2) {
                b1->BFlagSet(ChBody::BodyFlag::COULDSLEEP, false);
            }
            if (could_sleep2 && !(sleep1 || could_sleep1) && !ground1) {
                b2->BFlagSet(ChBody::BodyFlag::COULDSLEEP, false);
            }
            someone_sleeps = sleep1 | sleep2 | someone_sleeps;

            return true;  // to continue scanning contacts
        }

        // Data
        bool someone_sleeps;
        bool need_Setup_A;
    };

    auto my_waker = chrono_types::make_shared<_wakeup_reporter_class>();
    my_waker->need_Setup_A = false;

    bool need_Setup_L = false;

    for (int i = 0; i < 1; i++)  //***TO DO*** reconfigurable number of wakeup cycles
    {
        my_waker->someone_sleeps = false;

        // scan all links and wake connected bodies
        for (auto& link : assembly.linklist) {
            if (auto Lpointer = std::dynamic_pointer_cast<ChLink>(link)) {
                if (Lpointer->IsRequiringWaking()) {
                    ChBody* b1 = dynamic_cast<ChBody*>(Lpointer->GetBody1());
                    ChBody* b2 = dynamic_cast<ChBody*>(Lpointer->GetBody2());
                    if (b1 && b2) {
                        bool sleep1 = b1->GetSleeping();
                        bool sleep2 = b2->GetSleeping();
                        bool could_sleep1 = b1->BFlagGet(ChBody::BodyFlag::COULDSLEEP);
                        bool could_sleep2 = b2->BFlagGet(ChBody::BodyFlag::COULDSLEEP);
                        if (sleep1 && !(sleep2 || could_sleep2)) {
                            b1->SetSleeping(false);
                            need_Setup_L = true;
                        }
                        if (sleep2 && !(sleep1 || could_sleep1)) {
                            b2->SetSleeping(false);
                            need_Setup_L = true;
                        }
                        if (could_sleep1 && !(sleep2 || could_sleep2)) {
                            b1->BFlagSet(ChBody::BodyFlag::COULDSLEEP, false);
                        }
                        if (could_sleep2 && !(sleep1 || could_sleep1)) {
                            b2->BFlagSet(ChBody::BodyFlag::COULDSLEEP, false);
                        }
                    }
                }
            }
        }

        // scan all contacts and wake neighboring bodies
        contact_container->ReportAllContacts(my_waker);

        // bailout wakeup cycle prematurely, if all bodies are not sleeping
        if (!my_waker->someone_sleeps)
            break;
    }

    /// If some body still must change from no sleep-> sleep, do it
    int need_Setup_B = 0;
    for (auto& body : assembly.bodylist) {
        if (body->BFlagGet(ChBody::BodyFlag::COULDSLEEP)) {
            body->SetSleeping(true);
            ++need_Setup_B;
        }
    }

    // if some body has been activated/deactivated because of sleep state changes,
    // the offsets and DOF counts must be updated:
    if (my_waker->need_Setup_A || need_Setup_B || need_Setup_L) {
        Setup();
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------
//  DESCRIPTOR BOOKKEEPING
// -----------------------------------------------------------------------------

void ChSystem::DescriptorPrepareInject(ChSystemDescriptor& mdescriptor) {
    mdescriptor.BeginInsertion();  // This resets the vectors of constr. and var. pointers.

    InjectConstraints(mdescriptor);
    InjectVariables(mdescriptor);
    InjectKRMmatrices(mdescriptor);

    mdescriptor.EndInsertion();
}

// -----------------------------------------------------------------------------

// SETUP
//
// Set all  offsets in position/speed global vectors, for all items.
// Count all bodies and links, etc, compute &set dof for statistics,
// allocates or reallocate bookkeeping data/vectors, if any,

void ChSystem::Setup() {
    CH_PROFILE("Setup");

    timer_setup.start();

    ncoords = 0;
    ncoords_w = 0;
    ndoc = 0;
    ndoc_w = 0;
    ndoc_w_C = 0;
    ndoc_w_D = 0;

    // Set up the underlying assembly (compute offsets of bodies, links, etc.)
    assembly.Setup();
    ncoords += assembly.ncoords;
    ncoords_w += assembly.ncoords_w;
    ndoc_w += assembly.ndoc_w;
    ndoc_w_C += assembly.ndoc_w_C;
    ndoc_w_D += assembly.ndoc_w_D;

    // Compute offsets for contact container
    contact_container->SetOffset_L(assembly.offset_L + ndoc_w);
    ndoc_w += contact_container->GetDOC();
    ndoc_w_C += contact_container->GetDOC_c();
    ndoc_w_D += contact_container->GetDOC_d();

    ndoc = ndoc_w + assembly.nbodies;  // number of constraints including quaternion constraints.
    nsysvars = ncoords + ndoc;         // total number of variables (coordinates + lagrangian multipliers)
    nsysvars_w = ncoords_w + ndoc_w;   // total number of variables (with 6 dof per body)

    ndof = ncoords - ndoc;  // number of degrees of freedom (approximate - does not consider constr. redundancy, etc)

    timer_setup.stop();

#ifdef _DEBUG
    // BOOKKEEPING SANITY CHECK
    // Test if the bookkeeping is properly aligned, at least for state gather/scatters,
    // by filling a marked vector, and see if some gaps or overlaps are remaining.

    bool check_bookkeeping = false;
    if (check_bookkeeping) {
        ChState test_x(GetNcoords_x(), this);
        ChStateDelta test_v(GetNcoords_w(), this);
        ChStateDelta test_a(GetNcoords_w(), this);
        ChVectorDynamic<> test_L(GetNconstr());
        double poison_x = -8888.888;
        double poison_v = -9999.999;
        double poison_a = -7777.777;
        double poison_L = 55555.555;
        double test_T;
        test_x.setConstant(poison_x);  // poison x
        test_v.setConstant(poison_v);  // poison v
        test_a.setConstant(poison_a);  // poison a
        test_L.setConstant(poison_L);  // poison L
        StateGather(test_x, test_v, test_T);
        StateGatherAcceleration(test_a);
        StateGatherReactions(test_L);
        for (int i = 0; i < test_x.size(); ++i)
            assert(test_x(i) != poison_x);  // if your debugger breaks here, some ChPhysicsItem has a wrong
                                            // implementation of offsets or DOFs for positions
        for (int i = 0; i < test_v.size(); ++i)
            assert(test_v(i) != poison_v);  // if your debugger breaks here, some ChPhysicsItem has a wrong
                                            // implementation of offsets or DOFs for velocities
        for (int i = 0; i < test_a.size(); ++i)
            assert(test_a(i) != poison_a);  // if your debugger breaks here, some ChPhysicsItem has a wrong
                                            // implementation of offsets or DOFs for accelerations
        for (int i = 0; i < test_L.size(); ++i)
            assert(test_L(i) != poison_L);  // if your debugger breaks here, some ChPhysicsItem has a wrong
                                            // implementation of offsets or DOFs for reaction forces
    }
#endif  // _DEBUG
}

// -----------------------------------------------------------------------------
// UPDATE
//
// - all physical items (bodies, links,etc.) are updated,
//   also updating their auxiliary variables (rot.matrices, etc.).
// - updates all forces  (automatic, as children of bodies)
// - updates all markers (automatic, as children of bodies).

void ChSystem::Update(double mytime, bool update_assets) {
    ch_time = mytime;
    assembly.ChTime = mytime;
    Update(update_assets);
}

void ChSystem::Update(bool update_assets) {
    CH_PROFILE("Update");

    if (!is_initialized)
        SetupInitial();

    timer_update.start();  // Timer for profiling

    // Update underlying assembly (recursively update sub objects bodies, links, etc)
    assembly.Update(update_assets);

    // Update all contacts, if any
    contact_container->Update(ch_time, update_assets);

    timer_update.stop();
}

void ChSystem::ForceUpdate() {
    is_updated = false;
}

void ChSystem::IntToDescriptor(const unsigned int off_v,
                               const ChStateDelta& v,
                               const ChVectorDynamic<>& R,
                               const unsigned int off_L,
                               const ChVectorDynamic<>& L,
                               const ChVectorDynamic<>& Qc) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.IntToDescriptor(off_v, v, R, off_L, L, Qc);

    // Use also on contact container:
    unsigned int displ_L = off_L - assembly.offset_L;
    unsigned int displ_v = off_v - assembly.offset_w;
    contact_container->IntToDescriptor(displ_v + contact_container->GetOffset_w(), v, R,
                                       displ_L + contact_container->GetOffset_L(), L, Qc);
}

void ChSystem::IntFromDescriptor(const unsigned int off_v,
                                 ChStateDelta& v,
                                 const unsigned int off_L,
                                 ChVectorDynamic<>& L) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.IntFromDescriptor(off_v, v, off_L, L);

    // Use also on contact container:
    unsigned int displ_L = off_L - assembly.offset_L;
    unsigned int displ_v = off_v - assembly.offset_w;
    contact_container->IntFromDescriptor(displ_v + contact_container->GetOffset_w(), v,
                                         displ_L + contact_container->GetOffset_L(), L);
}

// -----------------------------------------------------------------------------

void ChSystem::InjectVariables(ChSystemDescriptor& mdescriptor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.InjectVariables(mdescriptor);

    // Use also on contact container:
    contact_container->InjectVariables(mdescriptor);
}

void ChSystem::VariablesFbReset() {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.VariablesFbReset();

    // Use also on contact container:
    contact_container->VariablesFbReset();
}

void ChSystem::VariablesFbLoadForces(double factor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.VariablesFbLoadForces();

    // Use also on contact container:
    contact_container->VariablesFbLoadForces();
}

void ChSystem::VariablesFbIncrementMq() {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.VariablesFbIncrementMq();

    // Use also on contact container:
    contact_container->VariablesFbIncrementMq();
}

void ChSystem::VariablesQbLoadSpeed() {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.VariablesQbLoadSpeed();

    // Use also on contact container:
    contact_container->VariablesQbLoadSpeed();
}

void ChSystem::VariablesQbSetSpeed(double step) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.VariablesQbSetSpeed(step);

    // Use also on contact container:
    contact_container->VariablesQbSetSpeed(step);
}

void ChSystem::VariablesQbIncrementPosition(double dt_step) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.VariablesQbIncrementPosition(dt_step);

    // Use also on contact container:
    contact_container->VariablesQbIncrementPosition(dt_step);
}

void ChSystem::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.InjectConstraints(mdescriptor);

    // Use also on contact container:
    contact_container->InjectConstraints(mdescriptor);
}

void ChSystem::ConstraintsBiReset() {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.ConstraintsBiReset();

    // Use also on contact container:
    contact_container->ConstraintsBiReset();
}

void ChSystem::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);

    // Use also on contact container:
    contact_container->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
}

void ChSystem::ConstraintsBiLoad_Ct(double factor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.ConstraintsBiLoad_Ct(factor);

    // Use also on contact container:
    contact_container->ConstraintsBiLoad_Ct(factor);
}

void ChSystem::ConstraintsBiLoad_Qc(double factor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.ConstraintsBiLoad_Qc(factor);

    // Use also on contact container:
    contact_container->ConstraintsBiLoad_Qc(factor);
}

void ChSystem::ConstraintsFbLoadForces(double factor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.ConstraintsFbLoadForces(factor);

    // Use also on contact container:
    contact_container->ConstraintsFbLoadForces(factor);
}

void ChSystem::ConstraintsLoadJacobians() {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.ConstraintsLoadJacobians();

    // Use also on contact container:
    contact_container->ConstraintsLoadJacobians();
}

void ChSystem::ConstraintsFetch_react(double factor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.ConstraintsFetch_react(factor);

    // Use also on contact container:
    contact_container->ConstraintsFetch_react(factor);
}

void ChSystem::InjectKRMmatrices(ChSystemDescriptor& mdescriptor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.InjectKRMmatrices(mdescriptor);

    // Use also on contact container:
    contact_container->InjectKRMmatrices(mdescriptor);
}

void ChSystem::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.KRMmatricesLoad(Kfactor, Rfactor, Mfactor);

    // Use also on contact container:
    contact_container->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
}

// -----------------------------------------------------------------------------
//    TIMESTEPPER INTERFACE
// -----------------------------------------------------------------------------

// From system to state y={x,v}
void ChSystem::StateGather(ChState& x, ChStateDelta& v, double& T) {
    unsigned int off_x = 0;
    unsigned int off_v = 0;

    // Operate on assembly items (bodies, links, etc.)
    assembly.IntStateGather(off_x, x, off_v, v, T);

    // Use also on contact container:
    unsigned int displ_x = off_x - assembly.offset_x;
    unsigned int displ_v = off_v - assembly.offset_w;
    contact_container->IntStateGather(displ_x + contact_container->GetOffset_x(), x,
                                      displ_v + contact_container->GetOffset_w(), v, T);

    T = ch_time;
}

// From state Y={x,v} to system.
void ChSystem::StateScatter(const ChState& x, const ChStateDelta& v, const double T, bool full_update) {
    unsigned int off_x = 0;
    unsigned int off_v = 0;
 
    // Let each object (bodies, links, etc.) in the assembly extract its own states.
    // Note that each object also performs an update
    assembly.IntStateScatter(off_x, x, off_v, v, T, full_update);

    // Use also on contact container:
    unsigned int displ_x = off_x - assembly.offset_x;
    unsigned int displ_v = off_v - assembly.offset_w;
    contact_container->IntStateScatter(displ_x + contact_container->GetOffset_x(), x,  //
                                       displ_v + contact_container->GetOffset_w(), v,  //
                                       T, full_update);

    ch_time = T;
}

// From system to state derivative (acceleration), some timesteppers might need last computed accel.
void ChSystem::StateGatherAcceleration(ChStateDelta& a) {
    unsigned int off_a = 0;

    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.IntStateGatherAcceleration(off_a, a);

    // Use also on contact container:
    unsigned int displ_a = off_a - assembly.offset_w;
    contact_container->IntStateGatherAcceleration(displ_a + contact_container->GetOffset_w(), a);
}

// From state derivative (acceleration) to system, sometimes might be needed
void ChSystem::StateScatterAcceleration(const ChStateDelta& a) {
    unsigned int off_a = 0;

    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.IntStateScatterAcceleration(off_a, a);

    // Use also on contact container:
    unsigned int displ_a = off_a - assembly.offset_w;
    contact_container->IntStateScatterAcceleration(displ_a + contact_container->GetOffset_w(), a);
}

// From system to reaction forces (last computed) - some timestepper might need this
void ChSystem::StateGatherReactions(ChVectorDynamic<>& L) {
    unsigned int off_L = 0;

    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.IntStateGatherReactions(off_L, L);

    // Use also on contact container:
    unsigned int displ_L = off_L - assembly.offset_L;
    contact_container->IntStateGatherReactions(displ_L + contact_container->GetOffset_L(), L);
}

// From reaction forces to system, ex. store last computed reactions in ChLink objects for plotting etc.
void ChSystem::StateScatterReactions(const ChVectorDynamic<>& L) {
    unsigned int off_L = 0;

    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.IntStateScatterReactions(off_L, L);

    // Use also on contact container:
    unsigned int displ_L = off_L - assembly.offset_L;
    contact_container->IntStateScatterReactions(displ_L + contact_container->GetOffset_L(), L);
}

// Perform x_new = x + dx    for x in    Y = {x, dx/dt}
// It takes care of the fact that x has quaternions, dx has angular vel etc.
// NOTE: the system is not updated automatically after the state increment, so one might
// need to call StateScatter() if needed.
void ChSystem::StateIncrementX(ChState& x_new, const ChState& x, const ChStateDelta& Dx) {
    unsigned int off_x = 0;
    unsigned int off_v = 0;

    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.IntStateIncrement(off_x, x_new, x, off_v, Dx);

    // Use also on contact container:
    unsigned int displ_x = off_x - assembly.offset_x;
    unsigned int displ_v = off_v - assembly.offset_w;
    contact_container->IntStateIncrement(displ_x + contact_container->GetOffset_x(), x_new, x,
                                         displ_v + contact_container->GetOffset_w(), Dx);
}

// Assuming a DAE of the form
//       M*a = F(x,v,t) + Cq'*L
//       C(x,t) = 0
// this function computes the solution of the change Du (in a or v or x) for a Newton
// iteration within an implicit integration scheme.
//  |Du| = [ G   Cq' ]^-1 * | R |
//  |DL|   [ Cq  0   ]      | Qc|
// for residual R and  G = [ c_a*M + c_v*dF/dv + c_x*dF/dx ]
// This function returns true if successful and false otherwise.
bool ChSystem::StateSolveCorrection(ChStateDelta& Dv,             // result: computed Dv
                                    ChVectorDynamic<>& L,         // result: computed lagrangian multipliers, if any
                                    const ChVectorDynamic<>& R,   // the R residual
                                    const ChVectorDynamic<>& Qc,  // the Qc residual
                                    const double c_a,             // the factor in c_a*M
                                    const double c_v,             // the factor in c_v*dF/dv
                                    const double c_x,             // the factor in c_x*dF/dx
                                    const ChState& x,             // current state, x part
                                    const ChStateDelta& v,        // current state, v part
                                    const double T,               // current time T
                                    bool force_state_scatter,     // if false, x,v and T are not scattered to the system
                                    bool full_update,             // if true, perform a full update during scatter
                                    bool force_setup              // if true, call the solver's Setup() function
) {
    CH_PROFILE("StateSolveCorrection");

    if (force_state_scatter)
        StateScatter(x, v, T, full_update);

    // R and Qc vectors  --> solver sparse solver structures  (also sets L and Dv to warmstart)
    IntToDescriptor(0, Dv, R, 0, L, Qc);

    // If the solver's Setup() must be called or if the solver's Solve() requires it,
    // fill the sparse system structures with information in G and Cq.
    if (force_setup || GetSolver()->SolveRequiresMatrix()) {
        timer_jacobian.start();

        // Cq  matrix
        ConstraintsLoadJacobians();

        // G matrix: M, K, R components
        if (c_a || c_v || c_x)
            KRMmatricesLoad(-c_x, -c_v, c_a);

        // For ChVariable objects without a ChKblock, just use the 'a' coefficient
        descriptor->SetMassFactor(c_a);

        timer_jacobian.stop();
    }

    // Diagnostics:
    if (dump_matrices) {
        const char* numformat = "%.12g";
        std::string sprefix = "solve_" + std::to_string(stepcount) + "_" + std::to_string(solvecount) + "_";

        descriptor->DumpLastMatrices(true, sprefix.c_str());
        descriptor->DumpLastMatrices(false, sprefix.c_str());

        chrono::ChStreamOutAsciiFile file_x((sprefix + "x_pre.dat").c_str());
        file_x.SetNumFormat(numformat);
        StreamOUTdenseMatlabFormat(x, file_x);

        chrono::ChStreamOutAsciiFile file_v((sprefix + "v_pre.dat").c_str());
        file_v.SetNumFormat(numformat);
        StreamOUTdenseMatlabFormat(v, file_v);
    }

    // If indicated, first perform a solver setup.
    // Return 'false' if the setup phase fails.
    if (force_setup) {
        timer_ls_setup.start();
        bool success = GetSolver()->Setup(*descriptor);
        timer_ls_setup.stop();
        setupcount++;
        if (!success)
            return false;
    }

    // Solve the problem
    // The solution is scattered in the provided system descriptor
    timer_ls_solve.start();
    GetSolver()->Solve(*descriptor);
    timer_ls_solve.stop();

    // Dv and L vectors  <-- sparse solver structures
    IntFromDescriptor(0, Dv, 0, L);

    // Diagnostics:
    if (dump_matrices) {
        const char* numformat = "%.12g";
        std::string sprefix = "solve_" + std::to_string(stepcount) + "_" + std::to_string(solvecount) + "_";

        chrono::ChStreamOutAsciiFile file_Dv((sprefix + "Dv.dat").c_str());
        file_Dv.SetNumFormat(numformat);
        StreamOUTdenseMatlabFormat(Dv, file_Dv);

        chrono::ChStreamOutAsciiFile file_L((sprefix + "L.dat").c_str());
        file_L.SetNumFormat(numformat);
        StreamOUTdenseMatlabFormat(L, file_L);

        // Just for diagnostic, dump also unscaled loads (forces,torques),
        // since the .._f.dat vector dumped in DumpLastMatrices() might contain scaled loads, and also +M*v
        ChVectorDynamic<> tempF(this->GetNcoords_v());
        tempF.setZero();
        LoadResidual_F(tempF, 1.0);
        chrono::ChStreamOutAsciiFile file_F((sprefix + "F_pre.dat").c_str());
        file_F.SetNumFormat(numformat);
        StreamOUTdenseMatlabFormat(tempF, file_F);
    }

    solvecount++;

    return true;
}

ChVector<> ChSystem::GetBodyAppliedForce(ChBody* body) {
    if (!is_initialized)
        return ChVector<>(0, 0, 0);

    if (!applied_forces_current) {
        applied_forces.setZero(this->GetNcoords_v());
        LoadResidual_F(applied_forces, 1.0);
        applied_forces_current = true;
    }
    return applied_forces.segment(body->Variables().GetOffset() + 0, 3);
}

ChVector<> ChSystem::GetBodyAppliedTorque(ChBody* body) {
    if (!is_initialized)
        return ChVector<>(0, 0, 0);

    if (!applied_forces_current) {
        applied_forces.setZero(this->GetNcoords_v());
        LoadResidual_F(applied_forces, 1.0);
        applied_forces_current = true;
    }
    return applied_forces.segment(body->Variables().GetOffset() + 3, 3);
}

// Increment a vector R with the term c*F:
//    R += c*F
void ChSystem::LoadResidual_F(ChVectorDynamic<>& R, const double c) {
    unsigned int off = 0;

    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.IntLoadResidual_F(off, R, c);

    // Use also on contact container:
    unsigned int displ_v = off - assembly.offset_w;
    contact_container->IntLoadResidual_F(displ_v + contact_container->GetOffset_w(), R, c);
}

// Increment a vector R with a term that has M multiplied a given vector w:
//    R += c*M*w
void ChSystem::LoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) {
    unsigned int off = 0;

    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.IntLoadResidual_Mv(off, R, w, c);

    // Use also on contact container:
    unsigned int displ_v = off - assembly.offset_w;
    contact_container->IntLoadResidual_Mv(displ_v + contact_container->GetOffset_w(), R, w, c);
}

// Increment a vectorR with the term Cq'*L:
//    R += c*Cq'*L
void ChSystem::LoadResidual_CqL(ChVectorDynamic<>& R, const ChVectorDynamic<>& L, const double c) {
    unsigned int off_L = 0;

    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.IntLoadResidual_CqL(off_L, R, L, c);

    // Use also on contact container:
    unsigned int displ_L = off_L - assembly.offset_L;
    contact_container->IntLoadResidual_CqL(displ_L + contact_container->GetOffset_L(), R, L, c);
}

// Increment a vector Qc with the term C:
//    Qc += c*C
void ChSystem::LoadConstraint_C(ChVectorDynamic<>& Qc,  // result: the Qc residual, Qc += c*C
                                const double c,         // a scaling factor
                                const bool do_clamp,    // enable optional clamping of Qc
                                const double clamp      // clamping value
) {
    unsigned int off_L = 0;

    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.IntLoadConstraint_C(off_L, Qc, c, do_clamp, clamp);

    // Use also on contact container:
    unsigned int displ_L = off_L - assembly.offset_L;
    contact_container->IntLoadConstraint_C(displ_L + contact_container->GetOffset_L(), Qc, c, do_clamp, clamp);
}

// Increment a vector Qc with the term Ct = partial derivative dC/dt:
//    Qc += c*Ct
void ChSystem::LoadConstraint_Ct(ChVectorDynamic<>& Qc, const double c) {
    unsigned int off_L = 0;

    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.IntLoadConstraint_Ct(off_L, Qc, c);

    // Use also on contact container:
    unsigned int displ_L = off_L - assembly.offset_L;
    contact_container->IntLoadConstraint_Ct(displ_L + contact_container->GetOffset_L(), Qc, c);
}

// -----------------------------------------------------------------------------
//   COLLISION OPERATIONS
// -----------------------------------------------------------------------------

int ChSystem::GetNcontacts() {
    return contact_container->GetNcontacts();
}

double ChSystem::ComputeCollisions() {
    CH_PROFILE("ComputeCollisions");

    double mretC = 0.0;

    timer_collision.start();

    // Update all positions of collision models: delegate this to the ChAssembly
    assembly.SyncCollisionModels();

    // Perform the collision detection ( broadphase and narrowphase )
    collision_system->Run();

    // Report and store contacts and/or proximities, if there are some
    // containers in the physic system. The default contact container
    // for ChBody and ChParticles is used always.
    {
        CH_PROFILE("ReportContacts");

        collision_system->ReportContacts(contact_container.get());

        for (auto& item : assembly.otherphysicslist) {
            if (auto mcontactcontainer = std::dynamic_pointer_cast<ChContactContainer>(item)) {
                // collision_system->ReportContacts(mcontactcontainer.get()); 
                // ***TEST*** if one wants to populate a ChContactContainer this would clear it anyway...
            }

            if (auto mproximitycontainer = std::dynamic_pointer_cast<ChProximityContainer>(item)) {
                collision_system->ReportProximities(mproximitycontainer.get());
            }
        }
    }

    // Invoke the custom collision callbacks (if any). These can potentially add
    // additional contacts to the contact container.
    for (size_t ic = 0; ic < collision_callbacks.size(); ic++)
        collision_callbacks[ic]->OnCustomCollision(this);

    // Cache the total number of contacts
    ncontacts = contact_container->GetNcontacts();

    timer_collision.stop();

    return mretC;
}

// =============================================================================
//   PHYSICAL OPERATIONS
// =============================================================================

void ChSystem::GetMassMatrix(ChSparseMatrix* M) {
    // IntToDescriptor(0, Dv, R, 0, L, Qc);
    // ConstraintsLoadJacobians();

    // Load all KRM matrices with the M part only
    KRMmatricesLoad(0, 0, 1.0);
    // For ChVariable objects without a ChKblock, but still with a mass:
    descriptor->SetMassFactor(1.0);

    // Fill system-level M matrix
    this->GetSystemDescriptor()->ConvertToMatrixForm(nullptr, M, nullptr, nullptr, nullptr, nullptr, false, false);
}

void ChSystem::GetStiffnessMatrix(ChSparseMatrix* K) {
    // IntToDescriptor(0, Dv, R, 0, L, Qc);
    // ConstraintsLoadJacobians();

    // Load all KRM matrices with the K part only
    this->KRMmatricesLoad(1.0, 0, 0);
    // For ChVariable objects without a ChKblock, but still with a mass:
    descriptor->SetMassFactor(0.0);

    // Fill system-level K matrix
    this->GetSystemDescriptor()->ConvertToMatrixForm(nullptr, K, nullptr, nullptr, nullptr, nullptr, false, false);
}

void ChSystem::GetDampingMatrix(ChSparseMatrix* R) {
    // IntToDescriptor(0, Dv, R, 0, L, Qc);
    // ConstraintsLoadJacobians();

    // Load all KRM matrices with the R part only
    this->KRMmatricesLoad(0, 1.0, 0);
    // For ChVariable objects without a ChKblock, but still with a mass:
    descriptor->SetMassFactor(0.0);

    // Fill system-level R matrix
    this->GetSystemDescriptor()->ConvertToMatrixForm(nullptr, R, nullptr, nullptr, nullptr, nullptr, false, false);
}

void ChSystem::GetConstraintJacobianMatrix(ChSparseMatrix* Cq) {
    // IntToDescriptor(0, Dv, R, 0, L, Qc);

    // Load all jacobian matrices
    this->ConstraintsLoadJacobians();

    // Fill system-level R matrix
    this->GetSystemDescriptor()->ConvertToMatrixForm(Cq, nullptr, nullptr, nullptr, nullptr, nullptr, false, false);
}

void ChSystem::DumpSystemMatrices(bool save_M, bool save_K, bool save_R, bool save_Cq, const char* path) {
    char filename[300];
    const char* numformat = "%.12g";

    if (save_M) {
        ChSparseMatrix mM;
        this->GetMassMatrix(&mM);
        sprintf(filename, "%s%s", path, "_M.dat");
        ChStreamOutAsciiFile file_M(filename);
        file_M.SetNumFormat(numformat);
        StreamOUTsparseMatlabFormat(mM, file_M);
    }
    if (save_K) {
        ChSparseMatrix mK;
        this->GetStiffnessMatrix(&mK);
        sprintf(filename, "%s%s", path, "_K.dat");
        ChStreamOutAsciiFile file_K(filename);
        file_K.SetNumFormat(numformat);
        StreamOUTsparseMatlabFormat(mK, file_K);
    }
    if (save_R) {
        ChSparseMatrix mR;
        this->GetDampingMatrix(&mR);
        sprintf(filename, "%s%s", path, "_R.dat");
        ChStreamOutAsciiFile file_R(filename);
        file_R.SetNumFormat(numformat);
        StreamOUTsparseMatlabFormat(mR, file_R);
    }
    if (save_Cq) {
        ChSparseMatrix mCq;
        this->GetConstraintJacobianMatrix(&mCq);
        sprintf(filename, "%s%s", path, "_Cq.dat");
        ChStreamOutAsciiFile file_Cq(filename);
        file_Cq.SetNumFormat(numformat);
        StreamOUTsparseMatlabFormat(mCq, file_Cq);
    }
}

// -----------------------------------------------------------------------------
//  PERFORM AN INTEGRATION STEP.  ----
//
//  Advances a single time step.
//
//  Note that time step can be modified if some variable-time stepper is used.
// -----------------------------------------------------------------------------

int ChSystem::DoStepDynamics(double step_size) {
    if (!is_initialized)
        SetupInitial();

    applied_forces_current = false;
    step = step_size;
    return Integrate_Y();
}

// -----------------------------------------------------------------------------
//  PERFORM INTEGRATION STEP  using pluggable timestepper
// -----------------------------------------------------------------------------

bool ChSystem::Integrate_Y() {
    CH_PROFILE("Integrate_Y");

    ResetTimers();

    timer_step.start();

    stepcount++;
    solvecount = 0;
    setupcount = 0;

    // Compute contacts and create contact constraints
    int ncontacts_old = ncontacts;
    ComputeCollisions();

    // Declare an NSC system as "out of date" if there are contacts
    if (GetContactMethod() == ChContactMethod::NSC && (ncontacts_old != 0 || ncontacts != 0))
        is_updated = false;

    // Counts dofs, number of constraints, statistics, etc.
    // Note: this must be invoked at all times (regardless of the flag is_updated), as various physics items may use
    // their own Setup to perform operations at the beginning of a step.
    Setup();

    // If needed, update everything. No need to update visualization assets here.
    if (!is_updated) {
        Update(false);
    }

    // Re-wake the bodies that cannot sleep because they are in contact with
    // some body that is not in sleep state.
    ManageSleepingBodies();

    // Prepare lists of variables and constraints.
    DescriptorPrepareInject(*descriptor);

    // No need to update counts and offsets, as already done by the above call (in ChSystemDescriptor::EndInsertion)
    ////descriptor->UpdateCountsAndOffsets();

    // Set some settings in timestepper object
    timestepper->SetQcDoClamp(true);
    timestepper->SetQcClamping(max_penetration_recovery_speed);
    if (std::dynamic_pointer_cast<ChTimestepperHHT>(timestepper) ||
        std::dynamic_pointer_cast<ChTimestepperNewmark>(timestepper))
        timestepper->SetQcDoClamp(false);

    // PERFORM TIME STEP HERE!
    {
        CH_PROFILE("Advance");
        timer_advance.start();
        timestepper->Advance(step);
        timer_advance.stop();
    }

    // Executes custom processing at the end of step
    CustomEndOfStep();

    // Call method to gather contact forces/torques in rigid bodies
    contact_container->ComputeContactForces();

    // Time elapsed for step
    timer_step.stop();

    // Tentatively mark system as unchanged (i.e., no updated necessary)
    is_updated = true;

    return true;
}

// -----------------------------------------------------------------------------
// **** SATISFY ALL CONSTRAINT EQUATIONS WITH NEWTON
// **** ITERATION, UNTIL TOLERANCE SATISFIED, THEN UPDATE
// **** THE "Y" STATE WITH SetY (WHICH AUTOMATICALLY UPDATES
// **** ALSO AUXILIARY MATRICES).
// -----------------------------------------------------------------------------

bool ChSystem::DoAssembly(int action) {
    if (!is_initialized)
        SetupInitial();

    applied_forces_current = false;

    solvecount = 0;
    setupcount = 0;

    Setup();
    Update();

    // Overwrite various parameters
    int new_max_iters = 300;        // if using an iterative solver
    double new_tolerance = 1e-10;   // if using an iterative solver
    double new_step = 1e-6;

    int old_max_iters = GetSolverMaxIterations();
    double old_tolerance = GetSolverTolerance();
    double old_step = GetStep();

    SetSolverMaxIterations(std::max(old_max_iters, new_max_iters));
    SetSolverTolerance(new_tolerance);
    SetStep(new_step);

    // Prepare lists of variables and constraints.
    DescriptorPrepareInject(*descriptor);

    ChAssemblyAnalysis manalysis(*this);
    manalysis.SetMaxAssemblyIters(GetMaxiter());

    // Perform analysis
    manalysis.AssemblyAnalysis(action, new_step);

    // Restore parameters
    SetSolverMaxIterations(old_max_iters);
    SetSolverTolerance(old_tolerance);
    SetStep(old_step);

    return true;
}

// -----------------------------------------------------------------------------
// **** PERFORM THE LINEAR STATIC ANALYSIS
// -----------------------------------------------------------------------------

bool ChSystem::DoStaticLinear() {
    if (!is_initialized)
        SetupInitial();

    applied_forces_current = false;

    solvecount = 0;
    setupcount = 0;

    Setup();
    Update();

    int old_maxsteps = GetSolverMaxIterations();
    SetSolverMaxIterations(std::max(old_maxsteps, 300));

    // Prepare lists of variables and constraints.
    DescriptorPrepareInject(*descriptor);

    ChStaticLinearAnalysis manalysis(*this);

    // Perform analysis
    manalysis.StaticAnalysis();

    SetSolverMaxIterations(old_maxsteps);

    bool dump_data = false;

    if (dump_data) {
        GetSystemDescriptor()->DumpLastMatrices();

        // optional check for correctness in result
        chrono::ChVectorDynamic<double> md;
        GetSystemDescriptor()->BuildDiVector(md);  // d={f;-b}

        chrono::ChVectorDynamic<double> mx;
        GetSystemDescriptor()->FromUnknownsToVector(mx, true);  // x ={q,-l}
        chrono::ChStreamOutAsciiFile file_x("dump_x.dat");
        StreamOUTdenseMatlabFormat(mx, file_x);

        chrono::ChVectorDynamic<double> mZx;
        GetSystemDescriptor()->SystemProduct(mZx, mx);  // Zx = Z*x

        GetLog() << "CHECK: norm of solver residual: ||Z*x-d|| -------------------\n";
        GetLog() << (mZx - md).lpNorm<Eigen::Infinity>() << "\n";
    }

    return true;
}

// -----------------------------------------------------------------------------
// **** PERFORM THE NONLINEAR STATIC ANALYSIS
// -----------------------------------------------------------------------------

bool ChSystem::DoStaticNonlinear(int nsteps, bool verbose) {
    if (!is_initialized)
        SetupInitial();

    applied_forces_current = false;

    solvecount = 0;
    setupcount = 0;

    Setup();
    Update();

    int old_maxsteps = GetSolverMaxIterations();
    SetSolverMaxIterations(std::max(old_maxsteps, 300));

    // Prepare lists of variables and constraints.
    DescriptorPrepareInject(*descriptor);

    ChStaticNonLinearAnalysis manalysis(*this);
    manalysis.SetMaxIterations(nsteps);
    manalysis.SetVerbose(verbose);

    // Perform analysis
    manalysis.StaticAnalysis();

    SetSolverMaxIterations(old_maxsteps);

    return true;
}

bool ChSystem::DoStaticNonlinear(std::shared_ptr<ChStaticNonLinearAnalysis> analysis) {
    if (!is_initialized)
        SetupInitial();

    applied_forces_current = false;

    Setup();
    Update();

    DescriptorPrepareInject(*descriptor);

    analysis->StaticAnalysis();

    return true;
}

// -----------------------------------------------------------------------------
// **** PERFORM THE STATIC ANALYSIS, FINDING THE STATIC
// **** EQUILIBRIUM OF THE SYSTEM, WITH ITERATIVE SOLUTION
// -----------------------------------------------------------------------------

bool ChSystem::DoStaticRelaxing(int nsteps) {
    if (!is_initialized)
        SetupInitial();

    applied_forces_current = false;

    solvecount = 0;
    setupcount = 0;

    int err = 0;

    if ((ncoords > 0) && (ndof >= 0)) {
        for (int m_iter = 0; m_iter < nsteps; m_iter++) {
            for (auto& body : assembly.bodylist) {
                // Set no body speed and no body accel.
                body->SetNoSpeedNoAcceleration();
            }
            for (auto& mesh : assembly.meshlist) {
                mesh->SetNoSpeedNoAcceleration();
            }
            for (auto& item : assembly.otherphysicslist) {
                item->SetNoSpeedNoAcceleration();
            }

            double undotime = GetChTime();
            DoFrameDynamics(undotime + (step * 1.8) * (((double)nsteps - (double)m_iter)) / (double)nsteps);
            ch_time = undotime;
        }

        for (auto& body : assembly.bodylist) {
            // Set no body speed and no body accel.
            body->SetNoSpeedNoAcceleration();
        }
        for (auto& mesh : assembly.meshlist) {
            mesh->SetNoSpeedNoAcceleration();
        }
        for (auto& item : assembly.otherphysicslist) {
            item->SetNoSpeedNoAcceleration();
        }
    }

    if (err) {
        last_err = true;
        GetLog() << "WARNING: some constraints may be redundant, but couldn't be eliminated \n";
    }
    return last_err;
}

// -----------------------------------------------------------------------------
// **** ---    THE KINEMATIC SIMULATION  ---
// **** PERFORM IK (INVERSE KINEMATICS) UNTIL THE END_TIME IS
// **** REACHED, STARTING FROM THE CURRENT TIME.
// -----------------------------------------------------------------------------

bool ChSystem::DoEntireKinematics(double end_time) {
    if (!is_initialized)
        SetupInitial();

    applied_forces_current = false;

    Setup();

    int action = AssemblyLevel::POSITION | AssemblyLevel::VELOCITY | AssemblyLevel::ACCELERATION;

    DoAssembly(action);
    // first check if there are redundant links (at least one NR cycle
    // even if the structure is already assembled)

    while (ch_time < end_time) {
        // Newton-Raphson iteration, closing constraints
        DoAssembly(action);

        if (last_err)
            return false;

        // Update time and repeat.
        ch_time += step;
    }

    return true;
}

// -----------------------------------------------------------------------------
// **** ---   THE DYNAMICAL SIMULATION   ---
// **** PERFORM EXPLICIT OR IMPLICIT INTEGRATION TO GET
// **** THE DYNAMICAL SIMULATION OF THE SYSTEM, UNTIL THE
// **** END_TIME IS REACHED.
// -----------------------------------------------------------------------------

bool ChSystem::DoEntireDynamics(double end_time) {
    if (!is_initialized)
        SetupInitial();

    applied_forces_current = false;

    Setup();

    // the system may have wrong layout, or too large
    // clearances in constraints, so it is better to
    // check for constraint violation each time the integration starts
    DoAssembly(AssemblyLevel::POSITION | AssemblyLevel::VELOCITY | AssemblyLevel::ACCELERATION);

    // Perform the integration steps until the end
    // time is reached.
    // All the updating (of Y, Y_dt and time) is done
    // automatically by Integrate()

    while (ch_time < end_time) {
        if (!Integrate_Y())
            break;  // >>> 1- single integration step,
                    //        updating Y, from t to t+dt.
        if (last_err)
            return false;
    }

    if (last_err)
        return false;
    return true;
}

// Perform the dynamical integration, from current ChTime to
// the specified end time, and terminating the integration exactly
// on the end time. Therefore, the step of integration may get a
// little increment/decrement to have the last step ending in end time.
// Note that this function can be used in iterations to provide results in
// a evenly spaced frames of time, even if the steps are changing.
// Also note that if the time step is higher than the time increment
// requested to reach end time, the step is lowered.

bool ChSystem::DoFrameDynamics(double end_time) {
    if (!is_initialized)
        SetupInitial();

    applied_forces_current = false;

    double old_step = 0;
    double left_time;
    bool restore_oldstep = false;
    int counter = 0;

    while (ch_time < end_time) {
        restore_oldstep = false;
        counter++;

        left_time = end_time - ch_time;

        if (left_time < 1e-12)
            break;  // - no integration if backward or null frame step.

        if (left_time < (1.3 * step))  // - step changed if too little frame step
        {
            old_step = step;
            step = left_time;
            restore_oldstep = true;
        }

        if (!Integrate_Y())
            break;  // ***  Single integration step,
                    // ***  updating Y, from t to t+dt.
                    // ***  This also changes local ChTime, and may change step

        if (last_err)
            break;
    }

    if (restore_oldstep)
        step = old_step;  // if timestep was changed to meet the end of frametime, restore pre-last (even for
                          // time-varying schemes)

    if (last_err)
        return false;
    return true;
}

// Performs the dynamical simulation, but using "frame integration"
// iteratively. The results are provided only at each frame (evenly
// spaced by "frame_step") rather than at each "step" (steps can be much
// more than frames, and they may be automatically changed by integrator).
// Moreover, the integration results shouldn't be dependent by the
// "frame_step" value (steps are performed anyway, like in normal "DoEntireDynamics"
// command).

bool ChSystem::DoEntireUniformDynamics(double end_time, double frame_step) {
    if (!is_initialized)
        SetupInitial();

    applied_forces_current = false;

    // the initial system may have wrong layout, or too large clearances in constraints.
    Setup();
    DoAssembly(AssemblyLevel::POSITION | AssemblyLevel::VELOCITY | AssemblyLevel::ACCELERATION);

    while (ch_time < end_time) {
        double goto_time = (ch_time + frame_step);
        if (!DoFrameDynamics(goto_time))
            return false;
    }

    return true;
}

// Like DoFrameDynamics, but performs kinematics instead of dynamics

bool ChSystem::DoFrameKinematics(double end_time) {
    if (!is_initialized)
        SetupInitial();

    applied_forces_current = false;

    double old_step = 0;
    double left_time;
    int restore_oldstep;
    int counter = 0;

    ////double frame_step = (end_time - ch_time);

    while (ch_time < end_time) {
        restore_oldstep = false;
        counter++;

        left_time = end_time - ch_time;

        if (left_time < 0.000000001)
            break;  // - no kinematics for backward

        if (left_time < (1.3 * step))  // - step changed if too little frame step
        {
            old_step = step;
            step = left_time;
            restore_oldstep = true;
        }

        // Newton Raphson kinematic equations solver
        DoAssembly(AssemblyLevel::POSITION | AssemblyLevel::VELOCITY | AssemblyLevel::ACCELERATION);

        if (last_err)
            return false;

        ch_time += step;

        if (restore_oldstep)
            step = old_step;  // if timestep was changed to meet the end of frametime
    }

    return true;
}

bool ChSystem::DoStepKinematics(double step_size) {
    if (!is_initialized)
        SetupInitial();

    applied_forces_current = false;

    ch_time += step_size;

    Update();

    // Newton Raphson kinematic equations solver
    DoAssembly(AssemblyLevel::POSITION | AssemblyLevel::VELOCITY | AssemblyLevel::ACCELERATION);

    if (last_err)
        return false;

    return true;
}

// Full assembly -computes also forces-
bool ChSystem::DoFullAssembly() {
    DoAssembly(AssemblyLevel::POSITION | AssemblyLevel::VELOCITY | AssemblyLevel::ACCELERATION);

    return last_err;
}

// -----------------------------------------------------------------------------
//  STREAMING - FILE HANDLING

void ChSystem::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSystem>();

    // serialize underlying assembly
    assembly.ArchiveOUT(marchive);

    // serialize all member data:

    marchive << CHNVP(contact_container);

    marchive << CHNVP(G_acc);
    marchive << CHNVP(ch_time);
    marchive << CHNVP(step);
    marchive << CHNVP(step_min);
    marchive << CHNVP(step_max);
    marchive << CHNVP(stepcount);
    marchive << CHNVP(dump_matrices);

    marchive << CHNVP(tol_force);
    marchive << CHNVP(maxiter);
    marchive << CHNVP(use_sleeping);

    marchive << CHNVP(descriptor);
    marchive << CHNVP(solver);

    marchive << CHNVP(min_bounce_speed);
    marchive << CHNVP(max_penetration_recovery_speed);

    marchive << CHNVP(collision_system);  // ChCollisionSystem should implement class factory for abstract create

    marchive << CHNVP(timestepper);  // ChTimestepper should implement class factory for abstract create

    //***TODO*** complete...
}

// Method to allow de serialization of transient data from archives.
void ChSystem::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChSystem>();

    // deserialize unerlying assembly
    assembly.ArchiveIN(marchive);

    // stream in all member data:

    marchive >> CHNVP(contact_container);

    marchive >> CHNVP(G_acc);
    marchive >> CHNVP(ch_time);
    marchive >> CHNVP(step);
    marchive >> CHNVP(step_min);
    marchive >> CHNVP(step_max);
    marchive >> CHNVP(stepcount);
    marchive >> CHNVP(dump_matrices);

    marchive >> CHNVP(tol_force);
    marchive >> CHNVP(maxiter);
    marchive >> CHNVP(use_sleeping);

    marchive >> CHNVP(descriptor);
    marchive >> CHNVP(solver);

    marchive >> CHNVP(min_bounce_speed);
    marchive >> CHNVP(max_penetration_recovery_speed);

    marchive >> CHNVP(collision_system);  // ChCollisionSystem should implement class factory for abstract create

    marchive >> CHNVP(timestepper);  // ChTimestepper should implement class factory for abstract create
    timestepper->SetIntegrable(this);

    //***TODO*** complete...

    //  Rebuild link pointers to markers
    Reference_LM_byID();

    // Recompute statistics, offsets, etc.
    Setup();
}

#define CH_CHUNK_START "Chrono binary file start"
#define CH_CHUNK_END "Chrono binary file end"

int ChSystem::FileProcessChR(ChStreamInBinary& m_file) {
    std::string mchunk;

    m_file >> mchunk;
    if (mchunk != CH_CHUNK_START)
        throw ChException("Not a ChR data file.");

    // StreamINall(m_file);

    m_file >> mchunk;
    if (mchunk != CH_CHUNK_END)
        throw ChException("The end of ChR data file is badly formatted.");

    return 1;
}

int ChSystem::FileWriteChR(ChStreamOutBinary& m_file) {
    m_file << CH_CHUNK_START;

    // StreamOUTall(m_file);

    m_file << CH_CHUNK_END;

    return 1;
}

}  // end namespace chrono
