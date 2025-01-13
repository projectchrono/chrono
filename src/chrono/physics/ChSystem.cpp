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
#include <iomanip>
#include <fstream>

#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#ifdef CHRONO_COLLISION
    #include "chrono/collision/multicore/ChCollisionSystemMulticore.h"
#endif
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/physics/ChProximityContainer.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/solver/ChSolverAPGD.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/solver/ChSolverPJacobi.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/solver/ChSolverPSSOR.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/utils/ChProfiler.h"
#include "chrono/physics/ChLinkMate.h"

namespace chrono {

// -----------------------------------------------------------------------------
// CLASS FOR PHYSICAL SYSTEM
// -----------------------------------------------------------------------------

ChSystem::ChSystem()
    : G_acc(ChVector3d(0, -9.8, 0)),
      is_initialized(false),
      is_updated(false),
      m_num_coords_pos(0),
      m_num_coords_vel(0),
      m_num_constr(0),
      m_num_constr_bil(0),
      m_num_constr_uni(0),
      ch_time(0),
      m_RTF(0),
      step(0.04),
      use_sleeping(false),
      max_penetration_recovery_speed(0.6),
      stepcount(0),
      setupcount(0),
      solvecount(0),
      output_dir("."),
      write_matrix(false),
      ncontacts(0),
      composition_strategy(new ChContactMaterialCompositionStrategy),
      collision_system(nullptr),
      visual_system(nullptr),
      nthreads_chrono(1),
      nthreads_eigen(1),
      nthreads_collision(1),
      applied_forces_current(false) {
    assembly.system = this;

    // Set default collision envelope and margin
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.03);
    ChCollisionModel::SetDefaultSuggestedMargin(0.01);

    // Set default timestepper
    timestepper = chrono_types::make_shared<ChTimestepperEulerImplicitLinearized>(this);
}

ChSystem::ChSystem(const ChSystem& other) : m_RTF(0), collision_system(nullptr), visual_system(nullptr) {
    // Required by ChAssembly
    assembly = other.assembly;
    assembly.system = this;

    G_acc = other.G_acc;
    m_num_coords_pos = other.m_num_coords_pos;
    m_num_coords_vel = other.m_num_coords_vel;
    m_num_constr = other.m_num_constr;
    m_num_constr_bil = other.m_num_constr_bil;
    m_num_constr_uni = other.m_num_constr_uni;
    ch_time = other.ch_time;
    step = other.step;
    stepcount = other.stepcount;
    solvecount = other.solvecount;
    setupcount = other.setupcount;
    write_matrix = other.write_matrix;
    output_dir = other.output_dir;
    SetTimestepperType(other.GetTimestepperType());
    nthreads_chrono = other.nthreads_chrono;
    nthreads_eigen = other.nthreads_eigen;
    nthreads_collision = other.nthreads_collision;
    is_initialized = false;
    is_updated = false;
    applied_forces_current = false;

    max_penetration_recovery_speed = other.max_penetration_recovery_speed;
    SetSolverType(other.GetSolverType());
    use_sleeping = other.use_sleeping;

    ncontacts = other.ncontacts;

    collision_callbacks = other.collision_callbacks;
}

ChSystem::~ChSystem() {
    Clear();
}

void ChSystem::Clear() {
    assembly.Clear();

    if (visual_system)
        visual_system->OnClear(this);

    // contact_container->RemoveAllContacts();

    // ResetTimers();
}

// -----------------------------------------------------------------------------

void ChSystem::AddBody(std::shared_ptr<ChBody> body) {
    body->index = static_cast<unsigned int>(GetBodies().size());
    assembly.AddBody(body);
    body->SetSystem(this);
}

void ChSystem::AddShaft(std::shared_ptr<ChShaft> shaft) {
    shaft->index = static_cast<unsigned int>(GetShafts().size());
    assembly.AddShaft(shaft);
    shaft->SetSystem(this);
}

void ChSystem::AddLink(std::shared_ptr<ChLinkBase> link) {
    assembly.AddLink(link);
    link->SetSystem(this);
}

void ChSystem::AddMesh(std::shared_ptr<fea::ChMesh> mesh) {
    assembly.AddMesh(mesh);
    mesh->SetSystem(this);
}

void ChSystem::AddOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item) {
    assembly.AddOtherPhysicsItem(item);
    item->SetSystem(this);
}

void ChSystem::RemoveBody(std::shared_ptr<ChBody> body) {
    if (collision_system)
        body->RemoveCollisionModelsFromSystem(collision_system.get());
    assembly.RemoveBody(body);
    body->SetSystem(nullptr);
}

void ChSystem::RemoveShaft(std::shared_ptr<ChShaft> shaft) {
    if (collision_system)
        shaft->RemoveCollisionModelsFromSystem(collision_system.get());
    assembly.RemoveShaft(shaft);
    shaft->SetSystem(nullptr);
}

void ChSystem::RemoveLink(std::shared_ptr<ChLinkBase> link) {
    if (collision_system)
        link->RemoveCollisionModelsFromSystem(collision_system.get());
    assembly.RemoveLink(link);
    link->SetSystem(nullptr);
}

void ChSystem::RemoveMesh(std::shared_ptr<fea::ChMesh> mesh) {
    if (collision_system)
        mesh->RemoveCollisionModelsFromSystem(collision_system.get());
    assembly.RemoveMesh(mesh);
    mesh->SetSystem(nullptr);
}

void ChSystem::RemoveOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item) {
    if (collision_system)
        item->RemoveCollisionModelsFromSystem(collision_system.get());
    assembly.RemoveOtherPhysicsItem(item);
    item->SetSystem(nullptr);
}

// Add arbitrary physics item to the underlying assembly.
// NOTE: we cannot simply invoke ChAssembly::Add as this would not provide polymorphism!
void ChSystem::Add(std::shared_ptr<ChPhysicsItem> item) {
    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        AddBody(body);
        return;
    }

    if (auto shaft = std::dynamic_pointer_cast<ChShaft>(item)) {
        AddShaft(shaft);
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
    if (!item)
        return;

    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        RemoveBody(body);
        return;
    }

    if (auto shaft = std::dynamic_pointer_cast<ChShaft>(item)) {
        RemoveShaft(shaft);
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
        case ChSolver::Type::SPARSE_LU:
            solver = chrono_types::make_shared<ChSolverSparseLU>();
            break;
        case ChSolver::Type::SPARSE_QR:
            solver = chrono_types::make_shared<ChSolverSparseQR>();
            break;
        default:
            std::cout << "Unknown solver type. No solver was set." << std::endl;
            std::cout << "Use SetSolver()." << std::endl;
            break;
    }
}

void ChSystem::EnableSolverMatrixWrite(bool val, const std::string& out_dir) {
    write_matrix = val;
    output_dir = out_dir;
}

// -----------------------------------------------------------------------------

void ChSystem::RegisterCustomCollisionCallback(std::shared_ptr<CustomCollisionCallback> callback) {
    collision_callbacks.push_back(callback);
}

void ChSystem::UnregisterCustomCollisionCallback(std::shared_ptr<CustomCollisionCallback> callback) {
    auto itr = std::find(std::begin(collision_callbacks), std::end(collision_callbacks), callback);
    if (itr != collision_callbacks.end()) {
        collision_callbacks.erase(itr);
    }
}

// -----------------------------------------------------------------------------

void ChSystem::SetSystemDescriptor(std::shared_ptr<ChSystemDescriptor> newdescriptor) {
    assert(newdescriptor);
    descriptor = newdescriptor;
}

void ChSystem::SetSolver(std::shared_ptr<ChSolver> newsolver) {
    assert(newsolver);
    solver = newsolver;
}

void ChSystem::SetCollisionSystemType(ChCollisionSystem::Type type) {
    assert(assembly.GetNumBodiesActive() == 0);

    auto coll_sys_type = type;

#ifndef CHRONO_COLLISION
    if (type == ChCollisionSystem::Type::MULTICORE) {
        std::cerr << "Chrono was not built with Thrust support. Multicore collision system not available." << std::endl;
        std::cerr << "Using Bullet collision system." << std::endl;
        coll_sys_type = ChCollisionSystem::Type::BULLET;
    }
#endif

    switch (coll_sys_type) {
        case ChCollisionSystem::Type::BULLET:
            collision_system = chrono_types::make_shared<ChCollisionSystemBullet>();
            break;
        case ChCollisionSystem::Type::MULTICORE:
#ifdef CHRONO_COLLISION
            collision_system = chrono_types::make_shared<ChCollisionSystemMulticore>();
#endif
            break;
        default:
            std::cerr << "Unknown collision system type. No collision system was set." << std::endl;
            std::cerr << "Use SetCollisionSystem()." << std::endl;
            return;
    }

    collision_system->SetNumThreads(nthreads_collision);
    collision_system->SetSystem(this);
}

void ChSystem::SetCollisionSystem(std::shared_ptr<ChCollisionSystem> coll_system) {
    assert(coll_system);
    collision_system = coll_system;
    collision_system->SetNumThreads(nthreads_collision);
    collision_system->SetSystem(this);
}

void ChSystem::SetContactContainer(std::shared_ptr<ChContactContainer> container) {
    assert(container);
    contact_container = container;
    contact_container->SetSystem(this);
}

void ChSystem::SetMaterialCompositionStrategy(std::unique_ptr<ChContactMaterialCompositionStrategy>&& strategy) {
    composition_strategy = std::move(strategy);
}

void ChSystem::SetNumThreads(int num_threads_chrono, int num_threads_collision, int num_threads_eigen) {
    nthreads_chrono = std::max(1, num_threads_chrono);
    nthreads_collision = (num_threads_collision <= 0) ? num_threads_chrono : num_threads_collision;
    nthreads_eigen = (num_threads_eigen <= 0) ? num_threads_chrono : num_threads_eigen;

    if (collision_system)
        collision_system->SetNumThreads(nthreads_collision);
}

// -----------------------------------------------------------------------------

// Initial system setup before analysis. Must be called once the system construction is completed.
void ChSystem::Initialize() {
    if (is_initialized)
        return;

    // Set num threads for Eigen
    Eigen::setNbThreads(nthreads_eigen);

    assembly.SetupInitial();

    // Initialize the collision system, if one is attached
    if (collision_system) {
        collision_system->Initialize();
        collision_system->SetNumThreads(nthreads_collision);
    }

    // Initialize any visualization system to which this system is attached
    if (visual_system) {
        visual_system->Initialize();
    }

    is_initialized = true;
}

// -----------------------------------------------------------------------------
// PREFERENCES
// -----------------------------------------------------------------------------

void ChSystem::SetTimestepperType(ChTimestepper::Type type) {
    // Do nothing if changing to a CUSTOM timestepper.
    if (type == ChTimestepper::Type::CUSTOM)
        return;

    // Do nothing, if no change from current typestepper.
    if (timestepper && type == timestepper->GetType())
        return;

    // Plug in the new required timestepper
    // (the previous will be automatically deallocated thanks to shared pointers)
    switch (type) {
        case ChTimestepper::Type::EULER_IMPLICIT:
            timestepper = chrono_types::make_shared<ChTimestepperEulerImplicit>(this);
            std::static_pointer_cast<ChTimestepperEulerImplicit>(timestepper)->SetMaxIters(4);
            break;
        case ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED:
            timestepper = chrono_types::make_shared<ChTimestepperEulerImplicitLinearized>(this);
            break;
        case ChTimestepper::Type::EULER_IMPLICIT_PROJECTED:
            timestepper = chrono_types::make_shared<ChTimestepperEulerImplicitProjected>(this);
            break;
        case ChTimestepper::Type::TRAPEZOIDAL:
            timestepper = chrono_types::make_shared<ChTimestepperTrapezoidal>(this);
            std::static_pointer_cast<ChTimestepperTrapezoidal>(timestepper)->SetMaxIters(4);
            break;
        case ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED:
            timestepper = chrono_types::make_shared<ChTimestepperTrapezoidalLinearized>(this);
            std::static_pointer_cast<ChTimestepperTrapezoidalLinearized>(timestepper)->SetMaxIters(4);
            break;
        case ChTimestepper::Type::HHT:
            timestepper = chrono_types::make_shared<ChTimestepperHHT>(this);
            std::static_pointer_cast<ChTimestepperHHT>(timestepper)->SetMaxIters(4);
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
            throw std::invalid_argument("SetTimestepperType: timestepper not supported");
    }
}

bool ChSystem::ManageSleepingBodies() {
    if (!IsSleepingAllowed())
        return 0;

    // STEP 1:
    // See if some body could change from no sleep to sleep

    for (auto& body : assembly.bodylist) {
        // mark as 'could sleep' candidate
        body->TrySleeping();
    }

    // STEP 2:
    // See if some sleeping or potential sleeping body is touching a non sleeping one; if so, set to no sleep.

    // Make this class for iterating through contacts
    class _wakeup_reporter_class : public ChContactContainer::ReportContactCallback {
      public:
        // Callback, used to report contact points already added to the container.
        // If returns false, the contact scanning will be stopped.
        virtual bool OnReportContact(
            const ChVector3d& pA,             // get contact pA
            const ChVector3d& pB,             // get contact pB
            const ChMatrix33<>& plane_coord,  // get contact plane coordsystem (A column 'X' is contact normal)
            const double& distance,           // get contact distance
            const double& eff_radius,         // effective radius of curvature at contact
            const ChVector3d& react_forces,   // get react.forces (if already computed). In coordsystem 'plane_coord'
            const ChVector3d& react_torques,  // get react.torques, if rolling friction (if already computed).
            ChContactable* contactobjA,  // get model A (note: some containers may not support it and could be zero!)
            ChContactable* contactobjB   // get model B (note: some containers may not support it and could be zero!)
            ) override {
            if (!(contactobjA && contactobjB))
                return true;
            ChBody* b1 = dynamic_cast<ChBody*>(contactobjA);
            ChBody* b2 = dynamic_cast<ChBody*>(contactobjB);
            if (!(b1 && b2))
                return true;
            bool sleep1 = b1->IsSleeping();
            bool sleep2 = b2->IsSleeping();
            bool could_sleep1 = b1->candidate_sleeping;
            bool could_sleep2 = b2->candidate_sleeping;
            bool ground1 = b1->IsFixed();
            bool ground2 = b2->IsFixed();
            if (sleep1 && !(sleep2 || could_sleep2) && !ground2) {
                b1->SetSleeping(false);
                need_Setup_A = true;
            }
            if (sleep2 && !(sleep1 || could_sleep1) && !ground1) {
                b2->SetSleeping(false);
                need_Setup_A = true;
            }
            if (could_sleep1 && !(sleep2 || could_sleep2) && !ground2) {
                b1->candidate_sleeping = false;
            }
            if (could_sleep2 && !(sleep1 || could_sleep1) && !ground1) {
                b2->candidate_sleeping = false;
            }
            someone_sleeps = someone_sleeps || sleep1 || sleep2;

            return true;  // to continue scanning contacts
        }

        // Data
        bool someone_sleeps;
        bool need_Setup_A;
    };

    auto my_waker = chrono_types::make_shared<_wakeup_reporter_class>();
    my_waker->need_Setup_A = false;

    bool need_Setup_L = false;

    for (int i = 0; i < 1; i++)  //// TODO  reconfigurable number of wakeup cycles
    {
        my_waker->someone_sleeps = false;

        // scan all links and wake connected bodies
        for (auto& link : assembly.linklist) {
            if (auto Lpointer = std::dynamic_pointer_cast<ChLink>(link)) {
                if (Lpointer->IsRequiringWaking()) {
                    ChBody* b1 = dynamic_cast<ChBody*>(Lpointer->GetBody1());
                    ChBody* b2 = dynamic_cast<ChBody*>(Lpointer->GetBody2());
                    if (b1 && b2) {
                        bool sleep1 = b1->IsSleeping();
                        bool sleep2 = b2->IsSleeping();
                        bool could_sleep1 = b1->candidate_sleeping;
                        bool could_sleep2 = b2->candidate_sleeping;
                        if (sleep1 && !(sleep2 || could_sleep2)) {
                            b1->SetSleeping(false);
                            need_Setup_L = true;
                        }
                        if (sleep2 && !(sleep1 || could_sleep1)) {
                            b2->SetSleeping(false);
                            need_Setup_L = true;
                        }
                        if (could_sleep1 && !(sleep2 || could_sleep2)) {
                            b1->candidate_sleeping = false;
                        }
                        if (could_sleep2 && !(sleep1 || could_sleep1)) {
                            b2->candidate_sleeping = false;
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
        if (body->candidate_sleeping) {
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

void ChSystem::DescriptorPrepareInject(ChSystemDescriptor& sys_descriptor) {
    sys_descriptor.BeginInsertion();  // This resets the vectors of constr. and var. pointers.

    InjectConstraints(sys_descriptor);
    InjectVariables(sys_descriptor);
    InjectKRMMatrices(sys_descriptor);

    sys_descriptor.EndInsertion();
}

// -----------------------------------------------------------------------------

// SETUP
//
// Set all  offsets in position/speed global vectors, for all items.
// Count all bodies and links, etc, compute &set dof for statistics,
// allocates or reallocate bookkeeping data/vectors, if any,

void ChSystem::Setup() {
    if (!is_initialized)
        assembly.SetupInitial();

    CH_PROFILE("Setup");

    timer_setup.start();

    m_num_coords_pos = 0;
    m_num_coords_vel = 0;
    m_num_constr = 0;
    m_num_constr_bil = 0;
    m_num_constr_uni = 0;

    // Set up the underlying assembly (compute offsets of bodies, links, etc.)
    assembly.Setup();
    m_num_coords_pos += assembly.m_num_coords_pos;
    m_num_coords_vel += assembly.m_num_coords_vel;
    m_num_constr += assembly.m_num_constr;
    m_num_constr_bil += assembly.m_num_constr_bil;
    m_num_constr_uni += assembly.m_num_constr_uni;

    // Compute offsets for contact container
    contact_container->SetOffset_L(assembly.offset_L + m_num_constr);
    m_num_constr += contact_container->GetNumConstraints();
    m_num_constr_bil += contact_container->GetNumConstraintsBilateral();
    m_num_constr_uni += contact_container->GetNumConstraintsUnilateral();

    timer_setup.stop();

#ifdef _DEBUG
    // BOOKKEEPING SANITY CHECK
    // Test if the bookkeeping is properly aligned, at least for state gather/scatters,
    // by filling a marked vector, and see if some gaps or overlaps are remaining.

    bool check_bookkeeping = false;
    if (check_bookkeeping) {
        ChState test_x(GetNumCoordsPosLevel(), this);
        ChStateDelta test_v(GetNumCoordsVelLevel(), this);
        ChStateDelta test_a(GetNumCoordsVelLevel(), this);
        ChVectorDynamic<> test_L(GetNumConstraints());
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
        for (unsigned int i = 0; i < test_x.size(); ++i)
            assert(test_x(i) != poison_x);  // if your debugger breaks here, some ChPhysicsItem has a wrong
                                            // implementation of offsets or DOFs for positions
        for (unsigned int i = 0; i < test_v.size(); ++i)
            assert(test_v(i) != poison_v);  // if your debugger breaks here, some ChPhysicsItem has a wrong
                                            // implementation of offsets or DOFs for velocities
        for (unsigned int i = 0; i < test_a.size(); ++i)
            assert(test_a(i) != poison_a);  // if your debugger breaks here, some ChPhysicsItem has a wrong
                                            // implementation of offsets or DOFs for accelerations
        for (unsigned int i = 0; i < test_L.size(); ++i)
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

    Initialize();

    timer_update.start();  // Timer for profiling

    // Update underlying assembly (recursively update sub objects bodies, links, etc)
    assembly.Update(update_assets);

    // Update all contacts, if any
    contact_container->Update(ch_time, update_assets);

    // Update any attached visualization system only when also updating assets
    if (visual_system && update_assets)
        visual_system->OnUpdate(this);

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
    int displ_L = off_L - assembly.offset_L;
    int displ_v = off_v - assembly.offset_w;
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
    int displ_L = off_L - assembly.offset_L;
    int displ_v = off_v - assembly.offset_w;
    contact_container->IntFromDescriptor(displ_v + contact_container->GetOffset_w(), v,
                                         displ_L + contact_container->GetOffset_L(), L);
}

// -----------------------------------------------------------------------------

void ChSystem::InjectVariables(ChSystemDescriptor& sys_descriptor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.InjectVariables(sys_descriptor);

    // Use also on contact container:
    contact_container->InjectVariables(sys_descriptor);
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

void ChSystem::InjectConstraints(ChSystemDescriptor& sys_descriptor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.InjectConstraints(sys_descriptor);

    // Use also on contact container:
    contact_container->InjectConstraints(sys_descriptor);
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

void ChSystem::LoadConstraintJacobians() {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.LoadConstraintJacobians();

    // Use also on contact container:
    contact_container->LoadConstraintJacobians();
}

void ChSystem::ConstraintsFetch_react(double factor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.ConstraintsFetch_react(factor);

    // Use also on contact container:
    contact_container->ConstraintsFetch_react(factor);
}

void ChSystem::InjectKRMMatrices(ChSystemDescriptor& sys_descriptor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.InjectKRMMatrices(sys_descriptor);

    // Use also on contact container:
    contact_container->InjectKRMMatrices(sys_descriptor);
}

void ChSystem::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.LoadKRMMatrices(Kfactor, Rfactor, Mfactor);

    // Use also on contact container:
    contact_container->LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
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
//  | Du| = [ H   Cq' ]^-1 * | R |
//  |-Dl|   [ Cq  0   ]      |-Qc|
// for given residuals R and -Qc, and  H = [ c_a*M + c_v*dF/dv + c_x*dF/dx ]
// This function returns true if successful and false otherwise.
bool ChSystem::StateSolveCorrection(
    ChStateDelta& Dv,             // result: computed Dv
    ChVectorDynamic<>& Dl,        // result: computed Dl lagrangian multipliers, if any. Note sign.
    const ChVectorDynamic<>& R,   // the R residual
    const ChVectorDynamic<>& Qc,  // the Qc residual. Note sign.
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

    // R and Qc vectors  --> solver sparse solver structures  (also sets Dl and Dv to warmstart)
    IntToDescriptor(0, Dv, R, 0, Dl, Qc);

    // If the solver's Setup() must be called or if the solver's Solve() requires it,
    // fill the sparse system structures with information in G and Cq.
    if (force_setup || GetSolver()->SolveRequiresMatrix()) {
        timer_jacobian.start();

        // Cq  matrix
        LoadConstraintJacobians();

        // G matrix: M, K, R components
        if (c_a || c_v || c_x)
            LoadKRMMatrices(-c_x, -c_v, c_a);

        // For ChVariable objects without a ChKRMBlock, just use the 'a' coefficient
        descriptor->SetMassFactor(c_a);

        timer_jacobian.stop();
    }

    // Diagnostics:
    if (write_matrix) {
        std::string prefix = "solve_" + std::to_string(stepcount) + "_" + std::to_string(solvecount);

        if (std::dynamic_pointer_cast<ChIterativeSolver>(solver)) {
            descriptor->WriteMatrixSpmv(output_dir, prefix);
        } else {
            descriptor->WriteMatrix(output_dir, prefix);
            descriptor->WriteMatrixBlocks(output_dir, prefix);
        }

        std::ofstream file_x(output_dir + "/" + prefix + "_x_pre.dat");
        file_x << std::setprecision(12) << std::scientific;
        StreamOut(x, file_x);

        std::ofstream file_v(output_dir + "/" + prefix + "_v_pre.dat");
        file_v << std::setprecision(12) << std::scientific;
        StreamOut(v, file_v);
    }

    GetSolver()->EnableWrite(write_matrix, std::to_string(stepcount) + "_" + std::to_string(solvecount), output_dir);

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

    // Dv and Dl vectors  <-- sparse solver structures
    IntFromDescriptor(0, Dv, 0, Dl);

    // Diagnostics:
    if (write_matrix) {
        std::string prefix = "solve_" + std::to_string(stepcount) + "_" + std::to_string(solvecount) + "_";

        std::ofstream file_Dv(output_dir + "/" + prefix + "Dv.dat");
        file_Dv << std::setprecision(12) << std::scientific;
        StreamOut(Dv, file_Dv);

        std::ofstream file_Dl(output_dir + "/" + prefix + "Dl.dat");
        file_Dl << std::setprecision(12) << std::scientific;
        StreamOut(Dl, file_Dl);

        // Just for diagnostic, dump also unscaled loads (forces,torques),
        // since the .._f.dat vector dumped in WriteMatrixBlocks() might contain scaled loads, and also +M*v
        ChVectorDynamic<> tempF(this->GetNumCoordsVelLevel());
        tempF.setZero();
        LoadResidual_F(tempF, 1.0);
        std::ofstream file_F(output_dir + "/" + prefix + "F_pre.dat");
        file_F << std::setprecision(12) << std::scientific;
        StreamOut(tempF, file_F);
    }

    solvecount++;

    return true;
}

ChVector3d ChSystem::GetBodyAppliedForce(ChBody* body) {
    if (!is_initialized)
        return ChVector3d(0, 0, 0);

    if (!applied_forces_current) {
        applied_forces.setZero(this->GetNumCoordsVelLevel());
        LoadResidual_F(applied_forces, 1.0);
        applied_forces_current = true;
    }
    return applied_forces.segment(body->Variables().GetOffset() + 0, 3);
}

ChVector3d ChSystem::GetBodyAppliedTorque(ChBody* body) {
    if (!is_initialized)
        return ChVector3d(0, 0, 0);

    if (!applied_forces_current) {
        applied_forces.setZero(this->GetNumCoordsVelLevel());
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

// Adds the lumped mass to a Md vector, representing a mass diagonal matrix. Used by lumped explicit integrators.
// If mass lumping is impossible or approximate, adds scalar error to "error" parameter.
//    Md += c*diag(M)    or   Md += c*HRZ(M)
void ChSystem::LoadLumpedMass_Md(ChVectorDynamic<>& Md, double& err, const double c) {
    unsigned int off = 0;

    // Operate on assembly sub-objects (bodies, links, etc.)
    assembly.IntLoadLumpedMass_Md(off, Md, err, c);

    // Use also on contact container: [ does nothing anyway ]
    unsigned int displ_v = off - assembly.offset_w;
    contact_container->IntLoadLumpedMass_Md(displ_v + contact_container->GetOffset_w(), Md, err, c);
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

unsigned int ChSystem::GetNumContacts() {
    return contact_container->GetNumContacts();
}

unsigned int ChSystem::ComputeCollisions() {
    CH_PROFILE("ComputeCollisions");

    timer_collision.start();

    // Update all positions of collision models: delegate this to the ChAssembly
    assembly.SyncCollisionModels();

    // Perform the collision detection ( broadphase and narrowphase )
    collision_system->PreProcess();
    collision_system->Run();
    collision_system->PostProcess();

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
    ncontacts = contact_container->GetNumContacts();

    timer_collision.stop();

    return ncontacts;
}

// -----------------------------------------------------------------------------
// TIMERS
// -----------------------------------------------------------------------------

double ChSystem::GetTimerCollisionBroad() const {
    if (collision_system)
        return collision_system->GetTimerCollisionBroad();

    return 0;
}

double ChSystem::GetTimerCollisionNarrow() const {
    if (collision_system)
        return collision_system->GetTimerCollisionNarrow();

    return 0;
}

void ChSystem::ResetTimers() {
    timer_step.reset();
    timer_advance.reset();
    timer_ls_solve.reset();
    timer_ls_setup.reset();
    timer_jacobian.reset();
    timer_collision.reset();
    timer_setup.reset();
    timer_update.reset();
    if (collision_system)
        collision_system->ResetTimers();
}

// =============================================================================
//   PHYSICAL OPERATIONS
// =============================================================================

void ChSystem::GetMassMatrix(ChSparseMatrix& M) {
    // IntToDescriptor(0, Dv, R, 0, L, Qc);
    // LoadConstraintJacobians();

    // Load all KRM matrices with the M part only
    LoadKRMMatrices(0, 0, 1.0);
    // For ChVariable objects without a ChKRMBlock, but still with a mass:
    descriptor->SetMassFactor(1.0);

    // Fill system-level M matrix
    M.resize(descriptor->CountActiveVariables(), descriptor->CountActiveVariables());
    M.setZeroValues();
    descriptor->PasteMassKRMMatrixInto(M);
}

void ChSystem::GetStiffnessMatrix(ChSparseMatrix& K) {
    // IntToDescriptor(0, Dv, R, 0, L, Qc);
    // LoadConstraintJacobians();

    // Load all KRM matrices with the K part only
    this->LoadKRMMatrices(1.0, 0, 0);
    // For ChVariable objects without a ChKRMBlock, but still with a mass:
    descriptor->SetMassFactor(0.0);

    // Fill system-level K matrix
    K.resize(descriptor->CountActiveVariables(), descriptor->CountActiveVariables());
    K.setZeroValues();
    descriptor->PasteMassKRMMatrixInto(K);
}

void ChSystem::GetDampingMatrix(ChSparseMatrix& R) {
    // IntToDescriptor(0, Dv, R, 0, L, Qc);
    // LoadConstraintJacobians();

    // Load all KRM matrices with the R part only
    this->LoadKRMMatrices(0, 1.0, 0);
    // For ChVariable objects without a ChKRMBlock, but still with a mass:
    descriptor->SetMassFactor(0.0);

    // Fill system-level R matrix
    R.resize(descriptor->CountActiveVariables(), descriptor->CountActiveVariables());
    R.setZeroValues();
    descriptor->PasteMassKRMMatrixInto(R);
}

void ChSystem::GetConstraintJacobianMatrix(ChSparseMatrix& Cq) {
    // IntToDescriptor(0, Dv, R, 0, L, Qc);

    // Load all jacobian matrices
    this->LoadConstraintJacobians();

    // Fill system-level R matrix
    Cq.resize(descriptor->CountActiveConstraints(), descriptor->CountActiveVariables());
    Cq.setZeroValues();
    descriptor->PasteConstraintsJacobianMatrixInto(Cq);
}

void ChSystem::WriteSystemMatrices(bool save_M,
                                   bool save_K,
                                   bool save_R,
                                   bool save_Cq,
                                   const std::string& path,
                                   bool one_indexed) {
    // Prepare lists of variables and constraints, if not already prepared.
    DescriptorPrepareInject(*descriptor);

    if (save_M) {
        ChSparseMatrix mM;
        this->GetMassMatrix(mM);
        std::ofstream file_M(path + "_M.dat");
        file_M << std::setprecision(12) << std::scientific;
        StreamOut(mM, file_M, one_indexed);
    }
    if (save_K) {
        ChSparseMatrix mK;
        this->GetStiffnessMatrix(mK);
        std::ofstream file_K(path + "_K.dat");
        file_K << std::setprecision(12) << std::scientific;
        StreamOut(mK, file_K, one_indexed);
    }
    if (save_R) {
        ChSparseMatrix mR;
        this->GetDampingMatrix(mR);
        std::ofstream file_R(path + "_R.dat");
        file_R << std::setprecision(12) << std::scientific;
        StreamOut(mR, file_R, one_indexed);
    }
    if (save_Cq) {
        ChSparseMatrix mCq;
        this->GetConstraintJacobianMatrix(mCq);
        std::ofstream file_Cq(path + "_Cq.dat");
        file_Cq << std::setprecision(12) << std::scientific;
        StreamOut(mCq, file_Cq, one_indexed);
    }
}

/// Remove redundant constraints present in ChSystem through QR decomposition of constraints Jacobian matrix.
unsigned int ChSystem::RemoveRedundantConstraints(bool remove_links, double qr_tol, bool verbose) {
    // Setup system descriptor
    Setup();
    Update();
    DescriptorPrepareInject(*descriptor);

    ChSparseMatrix Cq;
    Cq.resize(descriptor->CountActiveConstraints(), descriptor->CountActiveVariables());
    Cq.setZeroValues();
    descriptor->PasteConstraintsJacobianMatrixInto(Cq, 0, 0, true);
    unsigned int Cq_rows = Cq.rows();

    ChSparseMatrix CqT = Cq.transpose();
    CqT.makeCompressed();

    // Perform QR decomposition on Cq to identify linearly-dependant rows (ie. redundant scalar constraint equations)
    Eigen::SparseQR<ChSparseMatrix, Eigen::COLAMDOrdering<int>> QR_dec;
    QR_dec.compute(CqT);

    double diag_val;
    unsigned int independent_row_count = 0;
    unsigned int max_diag = std::min(QR_dec.matrixR().rows(), QR_dec.matrixR().cols());
    for (unsigned int diag_sel = 0; diag_sel < max_diag; diag_sel++) {
        diag_val = QR_dec.matrixR().coeff(diag_sel, diag_sel);
        if (std::abs(diag_val) > qr_tol)
            independent_row_count++;
    }
    unsigned int dependent_row_count = Cq_rows - independent_row_count;
    ChVectorDynamic<int> redundant_constraints_idx = QR_dec.colsPermutation().indices().tail(dependent_row_count);

    if (verbose) {
        std::cout << "Removing redundant constraints." << std::endl;
        std::cout << "   QR decomposition rank: " << QR_dec.rank() << std::endl;
        std::cout << "   Number of constraints:" << std::endl;
        std::cout << "   - total (before removal): " << GetSystemDescriptor()->CountActiveConstraints() << std::endl;
        std::cout << "   - independent: " << independent_row_count << std::endl;
        std::cout << "   - dependent: " << Cq_rows - independent_row_count << std::endl;
        std::cout << "   Redundant constraints [Cq_global row idx , linkname, Cq_link row idx]:" << std::endl;
        for (auto c_sel = 0; c_sel < redundant_constraints_idx.size(); ++c_sel) {
            // find corresponding link
            std::shared_ptr<ChLinkBase> corr_link;
            for (const auto& link : GetLinks()) {
                if ((unsigned int)redundant_constraints_idx[c_sel] >= link->GetOffset_L() &&
                    (unsigned int)redundant_constraints_idx[c_sel] < link->GetOffset_L() + link->GetNumConstraints()) {
                    corr_link = link;
                    break;
                }
            }

            std::cout << "      - [" << redundant_constraints_idx[c_sel] << "]: " << corr_link->GetName() << "["
                      << (redundant_constraints_idx[c_sel] - corr_link->GetOffset_L()) << "/"
                      << corr_link->GetNumConstraints() << "]" << std::endl;
        }
    }

    // Remove identified redundant constraints
    std::vector<ChConstraint*>& constrList = GetSystemDescriptor()->GetConstraints();
    for (auto c_sel = 0; c_sel < redundant_constraints_idx.size(); c_sel++)
        constrList[redundant_constraints_idx[c_sel]]->SetRedundant(true);
    GetSystemDescriptor()->UpdateCountsAndOffsets();

    // Remove Degrees of Constraint to ChLinkMate constraints
    std::map<unsigned int, std::shared_ptr<ChLinkBase>> constr_map;  // store an ordered list of constraints offsets
    for (unsigned int i = 0; i < (unsigned int)GetLinks().size(); ++i) {
        // store the link offset
        auto link = GetLinks()[i];
        constr_map[link->GetOffset_L()] = link;
    }

    std::map<unsigned int, std::array<bool, 6>>
        constrnewmask_map;  // store the mask of ChLinkMate constraints (only if they are
                            // ChLinkMate!) that have redundant equations
    for (auto r_sel = 0; r_sel < redundant_constraints_idx.size(); ++r_sel) {
        // pick the constraint with redundant degrees of constraints
        auto constr_pair_mod = constr_map.upper_bound(redundant_constraints_idx[r_sel]);
        --constr_pair_mod;

        // if is of type ChLinkMate then *plan* to modify its mask
        if (auto constr_mod = std::dynamic_pointer_cast<ChLinkMateGeneric>(constr_pair_mod->second)) {
            auto sel_constr_offset = constr_mod->GetOffset_L();

            std::array<bool, 6> original_mask = {constr_mod->IsConstrainedX(),  constr_mod->IsConstrainedY(),
                                                 constr_mod->IsConstrainedZ(),  constr_mod->IsConstrainedRx(),
                                                 constr_mod->IsConstrainedRy(), constr_mod->IsConstrainedRz()};

            if (constrnewmask_map.find(sel_constr_offset) == constrnewmask_map.end())
                constrnewmask_map[sel_constr_offset] = original_mask;

            // find which degree of constraint is redundant within the link
            auto redundant_offset = redundant_constraints_idx[r_sel] - sel_constr_offset;
            unsigned int active_constraints = 0;
            for (unsigned int m_sel = 0; m_sel < original_mask.size(); ++m_sel) {
                if (original_mask[m_sel] == true) {
                    ++active_constraints;
                }
                if (active_constraints == redundant_offset + 1) {
                    constrnewmask_map[sel_constr_offset][m_sel] = false;
                    break;
                }
            }
        }
    }

    // Modify ChLinkMate constaints based on new mask
    for (auto constrnewmask_it = constrnewmask_map.begin(); constrnewmask_it != constrnewmask_map.end();
         ++constrnewmask_it) {
        std::dynamic_pointer_cast<ChLinkMateGeneric>(constr_map[constrnewmask_it->first])
            ->SetConstrainedCoords(constrnewmask_it->second[0], constrnewmask_it->second[1],
                                   constrnewmask_it->second[2], constrnewmask_it->second[3],
                                   constrnewmask_it->second[4], constrnewmask_it->second[5]);
    }

    // IMPORTANT: by modifying the mask of ChLinkMate, the underlying ChConstraints get deleted and offsets get
    // scrambled. Therefore, repopulate ChSystemDescriptor with updated scenario
    Setup();
    Update();
    DescriptorPrepareInject(*descriptor);

    if (verbose) {
        std::cout << "   New number of constraints: " << GetSystemDescriptor()->CountActiveConstraints() << std::endl;
    }

    // Actually REMOVE links now having DoC = 0 from system link list
    if (remove_links) {
        unsigned int i = 0;
        while (i < GetLinks().size()) {
            if (GetLinks()[i]->GetNumConstraints() == 0)
                RemoveLink(GetLinks()[i]);
            else
                ++i;
        }
    }

    // Return number of deactivated constraints
    return static_cast<unsigned int>(redundant_constraints_idx.size());
}

// -----------------------------------------------------------------------------
//  Forward dynamics analysis
// -----------------------------------------------------------------------------

bool ChSystem::AdvanceDynamics() {
    CH_PROFILE("AdvanceDynamics");

    ResetTimers();

    timer_step.start();

    stepcount++;
    solvecount = 0;
    setupcount = 0;

    // Let the visualization system (if any) perform setup operations
    if (visual_system)
        visual_system->OnSetup(this);

    // Compute contacts and create contact constraints
    unsigned int ncontacts_old = ncontacts;
    if (collision_system)
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
    if (timestepper->GetType() == ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED) {
        timestepper->Qc_do_clamp = true;
        timestepper->Qc_clamping = max_penetration_recovery_speed;
    } else {
        timestepper->Qc_do_clamp = false;
    }

    // Advance system state by one step
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

    // Update the run-time visualization system, if present
    if (visual_system)
        visual_system->OnUpdate(this);

    // Tentatively mark system as unchanged (i.e., no updated necessary)
    is_updated = true;

    return true;
}

int ChSystem::DoStepDynamics(double step_size) {
    Initialize();

    applied_forces_current = false;
    step = step_size;

    bool success = AdvanceDynamics();

    m_RTF = timer_step() / step;

    return success;
}

bool ChSystem::DoFrameDynamics(double frame_time, double step_size) {
    Initialize();

    applied_forces_current = false;
    step = step_size;
    bool success = true;

    while (ch_time < frame_time) {
        double left_time = frame_time - ch_time;

        if (left_time < 1e-12)
            break;

        if (left_time < 1.3 * step)
            step = left_time;

        if (!AdvanceDynamics()) {
            success = false;
            break;
        }
    }

    return success;
}

// -----------------------------------------------------------------------------
// System assembly
// -----------------------------------------------------------------------------

AssemblyAnalysis::ExitFlag ChSystem::DoAssembly(int action,
                                                int max_num_iterationsNR,
                                                double abstol_residualNR,
                                                double reltol_updateNR,
                                                double abstol_updateNR) {
    Initialize();

    applied_forces_current = false;

    solvecount = 0;
    setupcount = 0;

    Setup();
    Update();

    // Prepare lists of variables and constraints
    DescriptorPrepareInject(*descriptor);

    ChAssemblyAnalysis assembling(*this);
    assembling.SetMaxAssemblyIters(max_num_iterationsNR);

    // Perform analysis
    step = 1e-6;
    assembling.SetAbsToleranceResidual(abstol_residualNR);
    assembling.SetRelToleranceUpdate(reltol_updateNR);
    assembling.SetAbsToleranceUpdate(abstol_updateNR);
    AssemblyAnalysis::ExitFlag exit_flag = assembling.AssemblyAnalysis(action, step);

    // Update any attached visualization system
    if (visual_system)
        visual_system->OnUpdate(this);

    return exit_flag;
}

// -----------------------------------------------------------------------------
// Inverse kinematics analysis
// -----------------------------------------------------------------------------

AssemblyAnalysis::ExitFlag ChSystem::DoStepKinematics(double step_size) {
    Initialize();

    applied_forces_current = false;
    step = step_size;
    ch_time += step_size;

    Update();
    AssemblyAnalysis::ExitFlag exit_flag = DoAssembly(AssemblyAnalysis::Level::FULL);

    return exit_flag;
}

AssemblyAnalysis::ExitFlag ChSystem::DoFrameKinematics(double frame_time, double step_size) {
    Initialize();

    applied_forces_current = false;
    step = step_size;

    AssemblyAnalysis::ExitFlag exit_flag = AssemblyAnalysis::ExitFlag::SUCCESS;

    while (ch_time < frame_time) {
        double left_time = frame_time - ch_time;

        if (left_time < 1e-12)
            break;

        if (left_time < (1.3 * step))
            step = left_time;

        exit_flag = DoAssembly(AssemblyAnalysis::Level::FULL);

        if (exit_flag == AssemblyAnalysis::ExitFlag::NOT_CONVERGED) {
            break;
        }

        ch_time += step;
    }

    return exit_flag;
}

// -----------------------------------------------------------------------------
// Static analysis
// -----------------------------------------------------------------------------

bool ChSystem::DoStaticAnalysis(ChStaticAnalysis& analysis) {
    Initialize();

    applied_forces_current = false;

    solvecount = 0;
    setupcount = 0;

    Setup();
    Update();

    DescriptorPrepareInject(*descriptor);
    analysis.SetIntegrable(this);
    analysis.StaticAnalysis();

    // Update any attached visualization system
    if (visual_system)
        visual_system->OnUpdate(this);

    return true;
}

bool ChSystem::DoStaticLinear() {
    Initialize();

    applied_forces_current = false;

    solvecount = 0;
    setupcount = 0;

    Setup();
    Update();

    // Overwrite solver parameters (only if iterative)
    int new_max_iters = 300;
    int old_max_iters = 0;
    if (solver->IsIterative()) {
        old_max_iters = solver->AsIterative()->GetMaxIterations();
        solver->AsIterative()->SetMaxIterations(std::max(old_max_iters, new_max_iters));
    }

    // Prepare lists of variables and constraints.
    DescriptorPrepareInject(*descriptor);

    // Perform analysis
    ChStaticLinearAnalysis analysis;
    analysis.SetIntegrable(this);
    analysis.StaticAnalysis();

    // Restore solver parameters
    if (solver->IsIterative()) {
        solver->AsIterative()->SetMaxIterations(old_max_iters);
    }

    bool dump_data = false;

    if (dump_data) {
        descriptor->WriteMatrixBlocks("", "solve");

        // optional check for correctness in result
        ChVectorDynamic<double> md;
        GetSystemDescriptor()->BuildDiVector(md);  // d={f;-b}

        ChVectorDynamic<double> mx;
        GetSystemDescriptor()->FromUnknownsToVector(mx, true);  // x ={q,-l}
        std::ofstream file_x("solve_x.dat");
        StreamOut(mx, file_x);

        ChVectorDynamic<double> mZx;
        GetSystemDescriptor()->SystemProduct(mZx, mx);  // Zx = Z*x

        std::cout << "CHECK: norm of solver residual: ||Z*x-d|| -------------------" << std::endl;
        std::cout << (mZx - md).lpNorm<Eigen::Infinity>() << std::endl;
    }

    // Update any attached visualization system
    if (visual_system)
        visual_system->OnUpdate(this);

    return true;
}

bool ChSystem::DoStaticNonlinear(int nsteps, bool verbose) {
    Initialize();

    applied_forces_current = false;

    solvecount = 0;
    setupcount = 0;

    Setup();
    Update();

    // Overwrite solver parameters (only if iterative)
    int new_max_iters = 300;
    int old_max_iters = 0;
    if (solver->IsIterative()) {
        old_max_iters = solver->AsIterative()->GetMaxIterations();
        solver->AsIterative()->SetMaxIterations(std::max(old_max_iters, new_max_iters));
    }

    // Prepare lists of variables and constraints.
    DescriptorPrepareInject(*descriptor);

    // Perform analysis
    ChStaticNonLinearAnalysis analysis;
    analysis.SetIntegrable(this);
    analysis.SetMaxIterations(nsteps);
    analysis.SetVerbose(verbose);
    analysis.StaticAnalysis();

    // Restore solver parameters
    if (solver->IsIterative()) {
        solver->AsIterative()->SetMaxIterations(old_max_iters);
    }

    // Update any attached visualization system
    if (visual_system)
        visual_system->OnUpdate(this);

    return true;
}

bool ChSystem::DoStaticNonlinearRheonomic(
    int max_num_iterations,
    bool verbose,
    std::shared_ptr<ChStaticNonLinearRheonomicAnalysis::IterationCallback> callback) {
    Initialize();

    applied_forces_current = false;

    solvecount = 0;
    setupcount = 0;

    Setup();
    Update();

    // Overwrite solver parameters (only if iterative)
    int new_max_iters = 300;
    int old_max_iters = 0;
    if (solver->IsIterative()) {
        old_max_iters = solver->AsIterative()->GetMaxIterations();
        solver->AsIterative()->SetMaxIterations(std::max(old_max_iters, new_max_iters));
    }

    // Prepare lists of variables and constraints.
    DescriptorPrepareInject(*descriptor);

    // Perform analysis
    ChStaticNonLinearRheonomicAnalysis analysis;
    analysis.SetIntegrable(this);
    analysis.SetMaxIterations(max_num_iterations);
    analysis.SetVerbose(verbose);
    analysis.SetCallbackIterationBegin(callback);
    analysis.StaticAnalysis();

    // Restore solver parameters
    if (solver->IsIterative()) {
        solver->AsIterative()->SetMaxIterations(old_max_iters);
    }

    // Update any attached visualization system
    if (visual_system)
        visual_system->OnUpdate(this);

    return true;
}

bool ChSystem::DoStaticRelaxing(double step_size, int num_iterations) {
    Initialize();

    applied_forces_current = false;
    double current_time = ch_time;
    bool success = true;

    solvecount = 0;
    setupcount = 0;

    if (m_num_coords_pos == 0 || m_num_coords_pos < m_num_constr)
        return false;

    for (int i = 0; i < num_iterations; i++) {
        double frame_time = current_time + (step_size * 1.8) * (num_iterations - i) / (double)num_iterations;
        assembly.ForceToRest();
        success = DoFrameDynamics(frame_time, step_size);
        ch_time = current_time;
        if (!success)
            break;
    }

    assembly.ForceToRest();

    // Update any attached visualization system
    if (visual_system)
        visual_system->OnUpdate(this);

    return success;
}

// -----------------------------------------------------------------------------

void ChSystem::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChSystem>();

    // serialize underlying assembly
    archive_out << CHNVP(assembly);

    // serialize all member data:

    // archive_in >> CHNVP(contact_container); // created by the constructor

    archive_out << CHNVP(G_acc);
    archive_out << CHNVP(ch_time);
    archive_out << CHNVP(step);
    archive_out << CHNVP(stepcount);
    archive_out << CHNVP(write_matrix);

    archive_out << CHNVP(use_sleeping);

    archive_out << CHNVP(descriptor);
    archive_out << CHNVP(solver);

    archive_out << CHNVP(max_penetration_recovery_speed);

    archive_out << CHNVP(composition_strategy);

    // archive_out << CHNVP(timestepper);  // ChTimestepper should implement class factory for abstract create

    //// TODO  complete...
}

// Method to allow de serialization of transient data from archives.
void ChSystem::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChSystem>();

    // deserialize unerlying assembly
    archive_in >> CHNVP(assembly);

    // stream in all member data:

    // archive_in >> CHNVP(contact_container); // created by the constructor

    archive_in >> CHNVP(G_acc);
    archive_in >> CHNVP(ch_time);
    archive_in >> CHNVP(step);
    archive_in >> CHNVP(stepcount);
    archive_in >> CHNVP(write_matrix);

    archive_in >> CHNVP(use_sleeping);

    archive_in >> CHNVP(descriptor);
    archive_in >> CHNVP(solver);

    archive_in >> CHNVP(max_penetration_recovery_speed);

    archive_in >> CHNVP(composition_strategy);

    // archive_in >> CHNVP(timestepper);  // ChTimestepper should implement class factory for abstract create
    // timestepper->SetIntegrable(this);

    //// TODO  complete...

    // Recompute statistics, offsets, etc.
    Setup();
}

}  // end namespace chrono
