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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <algorithm>

#include "chrono/collision/ChCCollisionSystemBullet.h"
#include "chrono/collision/ChCModelBullet.h"
#include "chrono/parallel/ChOpenMP.h"
#include "chrono/physics/ChContactContainerDVI.h"
#include "chrono/physics/ChProximityContainerBase.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/solver/ChSolverAPGD.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/solver/ChSolverJacobi.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/solver/ChSolverPCG.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono/solver/ChSolverSOR.h"
#include "chrono/solver/ChSolverSORmultithread.h"
#include "chrono/solver/ChSolverSymmSOR.h"
#include "chrono/timestepper/ChStaticAnalysis.h"
#include "chrono/core/ChLinkedListMatrix.h"

using namespace chrono::collision;

namespace chrono {

// -----------------------------------------------------------------------------

// Class for iterating through all items of ChPhysicalItem that exist in
// a ChSystem.
// It will iterate through:
// - all ChBody          objects
// - all ChLink          objects
// - all ChPhysicalItems objects that were added, that were not ChBody or ChLink
// - the ChContactContainer object that manages all the contacts.
// Note that this iterator suffers a small overhead if compared to the use of
// three iterator cycles in succession (one over ChBody list, one over ChLink list, etc.)
// but it makes the code much more readable. Use it as follows:
//    IteratorAllPhysics my_iter(this);
//    while(my_iter.HasItem())
//    {
//        my_iter->....
//        ++my_iter;
//    }
class IteratorAllPhysics {
  public:
    IteratorAllPhysics(ChSystem* msys) : msystem(msys) { RewindToBegin(); }
    ~IteratorAllPhysics() {}

    void RewindToBegin() {
        list_bodies = msystem->Get_bodylist();
        node_body = list_bodies->begin();
        list_links = msystem->Get_linklist();
        node_link = list_links->begin();
        list_otherphysics = msystem->Get_otherphysicslist();
        node_otherphysics = list_otherphysics->begin();
        stage = 0;
        mptr = std::shared_ptr<ChPhysicsItem>();
        this->operator++();  // initialize with 1st available item
    }

    bool ReachedEnd() {
        if (stage == 9999)
            return true;
        return false;
    }
    bool HasItem() {
        if (mptr)
            return true;
        return false;
    }

    IteratorAllPhysics& operator=(const IteratorAllPhysics& other) {
        msystem = other.msystem;
        //...***TO DO***
        return (*this);
    }

    bool operator==(const IteratorAllPhysics& other) {
        return (msystem == other.msystem);  //...***TO COMPLETE***
    }

    bool operator!=(const IteratorAllPhysics& other) { return !(msystem == other.msystem); }

    IteratorAllPhysics& operator++() {
        switch (stage) {
            case 1: {
                node_body++;  // try next body
                if (node_body != list_bodies->end()) {
                    mptr = (*node_body);
                    return (*this);
                }
                break;
            }
            case 2: {
                node_link++;  // try next link
                if (node_link != list_links->end()) {
                    mptr = (*node_link);
                    return (*this);
                }
                break;
            }
            case 3: {
                node_otherphysics++;  // try next otherphysics
                if (node_otherphysics != list_otherphysics->end()) {
                    mptr = (*node_otherphysics);
                    return (*this);
                }
                break;
            }
            default:
                break;
        }
        // Something went wrong, some list was at the end, so jump to beginning of next list
        do {
            switch (stage) {
                case 0: {
                    stage = 1;
                    if (node_body != list_bodies->end()) {
                        mptr = (*node_body);
                        return (*this);
                    }
                    break;
                }
                case 1: {
                    stage = 2;
                    if (node_link != list_links->end()) {
                        mptr = (*node_link);
                        return (*this);
                    }
                    break;
                }
                case 2: {
                    stage = 3;
                    if (node_otherphysics != list_otherphysics->end()) {
                        mptr = (*node_otherphysics);
                        return (*this);
                    }
                    break;
                }
                case 3: {
                    stage = 4;
                    mptr = msystem->GetContactContainer();
                    return (*this);
                }
                case 4: {
                    stage = 9999;
                    mptr = std::shared_ptr<ChPhysicsItem>();
                    return (*this);
                }
            }  // end cases
        } while (true);

        return (*this);
    }

    std::shared_ptr<ChPhysicsItem> operator->() { return (mptr); }
    std::shared_ptr<ChPhysicsItem> operator*() { return (mptr); }

  private:
    std::vector<std::shared_ptr<ChBody> >::iterator node_body;
    std::vector<std::shared_ptr<ChBody> >* list_bodies;
    std::vector<std::shared_ptr<ChLink> >::iterator node_link;
    std::vector<std::shared_ptr<ChLink> >* list_links;
    std::vector<std::shared_ptr<ChPhysicsItem> >::iterator node_otherphysics;
    std::vector<std::shared_ptr<ChPhysicsItem> >* list_otherphysics;
    std::shared_ptr<ChPhysicsItem> mptr;
    int stage;
    ChSystem* msystem;
};

// -----------------------------------------------------------------------------
// CLASS FOR PHYSICAL SYSTEM
// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSystem)

ChSystem::ChSystem(unsigned int max_objects, double scene_size, bool init_sys)
    : ChAssembly(),
      end_time(1),
      step(0.04),
      step_min(0.002),
      step_max(0.04),
      tol(2e-4),
      tol_force(1e-3),
      maxiter(6),
      max_iter_solver_speed(30),
      max_iter_solver_stab(10),
      ncontacts(0),
      min_bounce_speed(0.15),
      max_penetration_recovery_speed(0.6),
      collisionpoint_callback(NULL),
      use_sleeping(false),
      G_acc(ChVector<>(0, -9.8, 0)),
      stepcount(0),
      solvecount(0),
      setupcount(0),
      dump_matrices(false),
      last_err(false)

{
    system = this;  // as needed by ChAssembly

    // Set default number of threads to be equal to number of available cores
    parallel_thread_number = CHOMPfunctions::GetNumProcs();

    // Set default contact container
    if (init_sys) {
        contact_container = std::make_shared<ChContactContainerDVI>();
        contact_container->SetSystem(this);
    }

    // Set default collision engine
    if (init_sys) {
        collision_system = std::make_shared<ChCollisionSystemBullet>(max_objects, scene_size);
    }

    // Set default collision envelope and margin.
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.03);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.01);

    // Set default timestepper.
    timestepper = std::make_shared<ChTimestepperEulerImplicitLinearized>(this);

    // Set default solver
    if (init_sys) {
        SetSolverType(ChSolver::Type::SYMMSOR);
    }
}

ChSystem::ChSystem(const ChSystem& other) : ChAssembly(other) {
    system = this;  // as needed by ChAssembly

    G_acc = other.G_acc;
    end_time = other.end_time;
    step = other.step;
    step_min = other.step_min;
    step_max = other.step_max;
    stepcount = other.stepcount;
    solvecount = other.solvecount;
    setupcount = other.setupcount;
    dump_matrices = other.dump_matrices;
    SetTimestepperType(other.GetTimestepperType());
    tol = other.tol;
    tol_force = other.tol_force;
    maxiter = other.maxiter;

    min_bounce_speed = other.min_bounce_speed;
    max_penetration_recovery_speed = other.max_penetration_recovery_speed;
    max_iter_solver_speed = other.max_iter_solver_speed;
    max_iter_solver_stab = other.max_iter_solver_stab;
    SetSolverType(GetSolverType());
    parallel_thread_number = other.parallel_thread_number;
    use_sleeping = other.use_sleeping;

    ncontacts = other.ncontacts;

    collision_callbacks = other.collision_callbacks;
    collisionpoint_callback = other.collisionpoint_callback;

    last_err = other.last_err;

    RemoveAllProbes();
    RemoveAllControls();
}

ChSystem::~ChSystem() {
    // Before proceeding, anticipate Clear(). This would be called also by base ChAssembly destructor, anyway, but
    // it would happen after this destructor, so the ith_body->SetSystem(0) in Clear() would not be able to remove
    // body's collision
    // models from the collision_system. Here it is possible, since the collision_system is still alive.
    Clear();

    RemoveAllProbes();
    RemoveAllControls();
}

void ChSystem::Clear() {
    // first the parent class data...
    ChAssembly::Clear();

    // contact_container->RemoveAllContacts();

    RemoveAllProbes();
    RemoveAllControls();

    // ResetTimers();
}

// -----------------------------------------------------------------------------
// Set/Get routines
// -----------------------------------------------------------------------------

void ChSystem::SetSolverType(ChSolver::Type type) {
    // Do nothing if changing to a CUSTOM solver.
    if (type == ChSolver::Type::CUSTOM)
        return;

    descriptor = std::make_shared<ChSystemDescriptor>();
    descriptor->SetNumThreads(parallel_thread_number);

    contact_container = std::make_shared<ChContactContainerDVI>();
    contact_container->SetSystem(this);

    switch (type) {
        case ChSolver::Type::SOR:
            solver_speed = std::make_shared<ChSolverSOR>();
            solver_stab = std::make_shared<ChSolverSOR>();
            break;
        case ChSolver::Type::SYMMSOR:
            solver_speed = std::make_shared<ChSolverSymmSOR>();
            solver_stab = std::make_shared<ChSolverSymmSOR>();
            break;
        case ChSolver::Type::JACOBI:
            solver_speed = std::make_shared<ChSolverJacobi>();
            solver_stab = std::make_shared<ChSolverJacobi>();
            break;
        case ChSolver::Type::SOR_MULTITHREAD:
            solver_speed = std::make_shared<ChSolverSORmultithread>("speedSolver", parallel_thread_number);
            solver_stab = std::make_shared<ChSolverSORmultithread>("posSolver", parallel_thread_number);
            break;
        case ChSolver::Type::PMINRES:
            solver_speed = std::make_shared<ChSolverPMINRES>();
            solver_stab = std::make_shared<ChSolverPMINRES>();
            break;
        case ChSolver::Type::BARZILAIBORWEIN:
            solver_speed = std::make_shared<ChSolverBB>();
            solver_stab = std::make_shared<ChSolverBB>();
            break;
        case ChSolver::Type::PCG:
            solver_speed = std::make_shared<ChSolverPCG>();
            solver_stab = std::make_shared<ChSolverPCG>();
            break;
        case ChSolver::Type::APGD:
            solver_speed = std::make_shared<ChSolverAPGD>();
            solver_stab = std::make_shared<ChSolverAPGD>();
            break;
        case ChSolver::Type::MINRES:
            solver_speed = std::make_shared<ChSolverMINRES>();
            solver_stab = std::make_shared<ChSolverMINRES>();
            break;
        default:
            solver_speed = std::make_shared<ChSolverSymmSOR>();
            solver_stab = std::make_shared<ChSolverSymmSOR>();
            break;
    }
}

std::shared_ptr<ChSolver> ChSystem::GetSolver() {
    // In case the solver is iterative, pre-configure it with the max. number of
    // iterations and with the convergence tolerance (convert the user-specified
    // tolerance for forces into a tolerance for impulses).
    if (auto iter_solver = std::dynamic_pointer_cast<ChIterativeSolver>(solver_speed)) {
        iter_solver->SetMaxIterations(GetMaxItersSolverSpeed());
        iter_solver->SetTolerance(tol_force * step);
    }

    return solver_speed;
}

std::shared_ptr<ChSolver> ChSystem::GetStabSolver() {
    // In case the solver is iterative, pre-configure it with the max. number of
    // iterations and with the convergence tolerance (convert the user-specified
    // tolerance for forces into a tolerance for impulses).
    if (auto iter_solver = std::dynamic_pointer_cast<ChIterativeSolver>(solver_stab)) {
        iter_solver->SetMaxIterations(GetMaxItersSolverSpeed());
        iter_solver->SetTolerance(tol_force * step);
    }

    return solver_stab;
}

void ChSystem::SetSolverWarmStarting(bool usewarm) {
    if (auto iter_solver_speed = std::dynamic_pointer_cast<ChIterativeSolver>(solver_speed)) {
        iter_solver_speed->SetWarmStart(usewarm);
    }
    if (auto iter_solver_stab = std::dynamic_pointer_cast<ChIterativeSolver>(solver_stab)) {
        iter_solver_stab->SetWarmStart(usewarm);
    }
}

bool ChSystem::GetSolverWarmStarting() const {
    if (auto iter_solver_speed = std::dynamic_pointer_cast<ChIterativeSolver>(solver_speed)) {
        return iter_solver_speed->GetWarmStart();
    }
    return false;
}

void ChSystem::SetSolverOverrelaxationParam(double momega) {
    if (auto iter_solver_speed = std::dynamic_pointer_cast<ChIterativeSolver>(solver_speed)) {
        iter_solver_speed->SetOmega(momega);
    }
    if (auto iter_solver_stab = std::dynamic_pointer_cast<ChIterativeSolver>(solver_stab)) {
        iter_solver_stab->SetOmega(momega);
    }
}

double ChSystem::GetSolverOverrelaxationParam() const {
    if (auto iter_solver_speed = std::dynamic_pointer_cast<ChIterativeSolver>(solver_speed)) {
        return iter_solver_speed->GetOmega();
    }
    return 1.0;
}

void ChSystem::SetSolverSharpnessParam(double momega) {
    if (auto iter_solver_speed = std::dynamic_pointer_cast<ChIterativeSolver>(solver_speed)) {
        iter_solver_speed->SetSharpnessLambda(momega);
    }
    if (auto iter_solver_stab = std::dynamic_pointer_cast<ChIterativeSolver>(solver_stab)) {
        iter_solver_stab->SetSharpnessLambda(momega);
    }
}

double ChSystem::GetSolverSharpnessParam() const {
    if (auto iter_solver_speed = std::dynamic_pointer_cast<ChIterativeSolver>(solver_speed)) {
        return iter_solver_speed->GetSharpnessLambda();
    }
    return 1.0;
}

void ChSystem::SetParallelThreadNumber(int mthreads) {
    if (mthreads < 1)
        mthreads = 1;

    parallel_thread_number = mthreads;

    descriptor->SetNumThreads(mthreads);

    if (solver_speed->GetType() == ChSolver::Type::SOR_MULTITHREAD) {
        std::static_pointer_cast<ChSolverSORmultithread>(solver_speed)->ChangeNumberOfThreads(mthreads);
        std::static_pointer_cast<ChSolverSORmultithread>(solver_stab)->ChangeNumberOfThreads(mthreads);
    }
}

// Plug-in components configuration

void ChSystem::SetSystemDescriptor(std::shared_ptr<ChSystemDescriptor> newdescriptor) {
    assert(newdescriptor);
    descriptor = newdescriptor;
}
void ChSystem::SetSolver(std::shared_ptr<ChSolver> newsolver) {
    assert(newsolver);
    solver_speed = newsolver;
}

void ChSystem::SetStabSolver(std::shared_ptr<ChSolver> newsolver) {
    assert(newsolver);
    solver_stab = newsolver;
}

void ChSystem::SetContactContainer(std::shared_ptr<ChContactContainerBase> container) {
    assert(container);
    contact_container = container;
    contact_container->SetSystem(this);
}

void ChSystem::SetCollisionSystem(std::shared_ptr<ChCollisionSystem> newcollsystem) {
    assert(GetNbodies() == 0);
    assert(newcollsystem);
    collision_system = newcollsystem;
}

// Initial system setup before analysis.
// This function must be called once the system construction is completed.
void ChSystem::SetupInitial() {
    for (int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->SetupInitial();
    }
    for (int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->SetupInitial();
    }
    for (int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->SetupInitial();
    }
}

// PROBE STUFF

int ChSystem::RecordAllProbes() {
    int pcount = 0;

    for (unsigned int ip = 0; ip < probelist.size(); ++ip) {
        probelist[ip]->Record(GetChTime());
    }

    return pcount;
}

int ChSystem::ResetAllProbes() {
    int pcount = 0;

    for (unsigned int ip = 0; ip < probelist.size(); ++ip) {
        probelist[ip]->Reset();
    }

    return pcount;
}

// CONTROLS STUFF

bool ChSystem::ExecuteControlsForUpdate() {
    for (unsigned int ip = 0; ip < controlslist.size(); ++ip) {
        if (!controlslist[ip]->ExecuteForUpdate())
            return false;
    }
    return true;
}

bool ChSystem::ExecuteControlsForStep() {
    for (unsigned int ip = 0; ip < controlslist.size(); ++ip) {
        if (!controlslist[ip]->ExecuteForStep())
            return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
// HIERARCHY HANDLERS
// -----------------------------------------------------------------------------

void ChSystem::AddProbe(const std::shared_ptr<ChProbe>& newprobe) {
    assert(std::find<std::vector<std::shared_ptr<ChProbe>>::iterator>(probelist.begin(), probelist.end(), newprobe) ==
           probelist.end());

    // newprobe->SetSystem (this);
    probelist.push_back(newprobe);
}

void ChSystem::AddControls(const std::shared_ptr<ChControls>& newcontrols) {
    assert(std::find<std::vector<std::shared_ptr<ChControls>>::iterator>(controlslist.begin(), controlslist.end(),
                                                                         newcontrols) == controlslist.end());

    // newcontrols->SetSystem (this);
    controlslist.push_back(newcontrols);
}

void ChSystem::RemoveAllProbes() {
    probelist.clear();
}

void ChSystem::RemoveAllControls() {
    controlslist.clear();
}

void ChSystem::Reference_LM_byID() {
    std::vector<std::shared_ptr<ChLink> > toremove;

    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        std::shared_ptr<ChLink> Lpointer = linklist[ip];
        if (auto malink = std::dynamic_pointer_cast<ChLinkMarkers>(Lpointer)) {
            std::shared_ptr<ChMarker> shm1 = SearchMarker(malink->GetMarkID1());
            std::shared_ptr<ChMarker> shm2 = SearchMarker(malink->GetMarkID2());
            ChMarker* mm1 = shm1.get();
            ChMarker* mm2 = shm1.get();
            malink->SetUpMarkers(mm1, mm2);
            if (mm1 && mm2) {
                Lpointer->SetValid(true);
            } else {
                Lpointer->SetValid(false);
                malink->SetUpMarkers(0, 0);  // note: marker IDs are maintained
                toremove.push_back(Lpointer);
            }
        }
    }
    for (int ir = 0; ir < toremove.size(); ++ir) {
        RemoveLink(toremove[ir]);
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
            timestepper = std::make_shared<ChTimestepperEulerImplicit>(this);
            std::static_pointer_cast<ChTimestepperEulerImplicit>(timestepper)->SetMaxiters(4);
            break;
        case ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED:
            timestepper = std::make_shared<ChTimestepperEulerImplicitLinearized>(this);
            break;
        case ChTimestepper::Type::EULER_IMPLICIT_PROJECTED:
            timestepper = std::make_shared<ChTimestepperEulerImplicitProjected>(this);
            break;
        case ChTimestepper::Type::TRAPEZOIDAL:
            timestepper = std::make_shared<ChTimestepperTrapezoidal>(this);
            std::static_pointer_cast<ChTimestepperTrapezoidal>(timestepper)->SetMaxiters(4);
            break;
        case ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED:
            timestepper = std::make_shared<ChTimestepperTrapezoidalLinearized>(this);
            std::static_pointer_cast<ChTimestepperTrapezoidalLinearized>(timestepper)->SetMaxiters(4);
            break;
        case ChTimestepper::Type::HHT:
            timestepper = std::make_shared<ChTimestepperHHT>(this);
            std::static_pointer_cast<ChTimestepperHHT>(timestepper)->SetMaxiters(4);
            break;
        case ChTimestepper::Type::HEUN:
            timestepper = std::make_shared<ChTimestepperHeun>(this);
            break;
        case ChTimestepper::Type::RUNGEKUTTA45:
            timestepper = std::make_shared<ChTimestepperRungeKuttaExpl>(this);
            break;
        case ChTimestepper::Type::EULER_EXPLICIT:
            timestepper = std::make_shared<ChTimestepperEulerExplIIorder>(this);
            break;
        case ChTimestepper::Type::LEAPFROG:
            timestepper = std::make_shared<ChTimestepperLeapfrog>(this);
            break;
        case ChTimestepper::Type::NEWMARK:
            timestepper = std::make_shared<ChTimestepperNewmark>(this);
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

    for (int ip = 0; ip < bodylist.size(); ++ip) {
        // mark as 'could sleep' candidate
        bodylist[ip]->TrySleeping();
    }

    // STEP 2:
    // See if some sleeping or potential sleeping body is touching a non sleeping one,
    // if so, set to no sleep.

    // Make this class for iterating through contacts

    class _wakeup_reporter_class : public ChReportContactCallback {
      public:
        // Callback, used to report contact points already added to the container.
        // This must be implemented by a child class of ChReportContactCallback.
        // If returns false, the contact scanning will be stopped.
        virtual bool ReportContactCallback(
            const ChVector<>& pA,             ///< get contact pA
            const ChVector<>& pB,             ///< get contact pB
            const ChMatrix33<>& plane_coord,  ///< get contact plane coordsystem (A column 'X' is contact normal)
            const double& distance,           ///< get contact distance
            const ChVector<>& react_forces,   ///< get react.forces (if already computed). In coordsystem 'plane_coord'
            const ChVector<>& react_torques,  ///< get react.torques, if rolling friction (if already computed).
            ChContactable* contactobjA,  ///< get model A (note: some containers may not support it and could be zero!)
            ChContactable* contactobjB   ///< get model B (note: some containers may not support it and could be zero!)
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

    _wakeup_reporter_class my_waker;
    my_waker.need_Setup_A = false;

    bool need_Setup_L = false;

    for (int i = 0; i < 1; i++)  //***TO DO*** reconfigurable number of wakeup cycles
    {
        my_waker.someone_sleeps = false;

        // scan all links and wake connected bodies
        for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
        {
            std::shared_ptr<ChLink> Lpointer = linklist[ip];

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

        // scan all contacts and wake neighbouring bodies
        contact_container->ReportAllContacts(&my_waker);

        // bailout wakeup cycle prematurely, if all bodies are not sleeping
        if (!my_waker.someone_sleeps)
            break;
    }

    /// If some body still must change from no sleep-> sleep, do it
    int need_Setup_B = 0;
    for (int ip = 0; ip < bodylist.size(); ++ip) {
        if (bodylist[ip]->BFlagGet(ChBody::BodyFlag::COULDSLEEP)) {
            bodylist[ip]->SetSleeping(true);
            ++need_Setup_B;
        }
    }

    // if some body has been activated/deactivated because of sleep state changes,
    // the offsets and DOF counts must be updated:
    if (my_waker.need_Setup_A || need_Setup_B || need_Setup_L) {
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
// CHPHYSICS ITEM INTERFACE
// -----------------------------------------------------------------------------

// SETUP
//
// Set all  offsets in position/speed global vectors, for all items.
// Count all bodies and links, etc, compute &set dof for statistics,
// allocates or reallocate bookkeeping data/vectors, if any,

void ChSystem::Setup() {
    // inherit the parent class (compute offsets of bodies, links, etc.)
    ChAssembly::Setup();

    // also compute offsets for contact container
    {
        contact_container->SetOffset_L(offset_L + ndoc_w);

        ndoc_w += contact_container->GetDOC();
        ndoc_w_C += contact_container->GetDOC_c();
        ndoc_w_D += contact_container->GetDOC_d();
    }

    ndoc = ndoc_w + nbodies;          // number of constraints including quaternion constraints.
    nsysvars = ncoords + ndoc;        // total number of variables (coordinates + lagrangian multipliers)
    nsysvars_w = ncoords_w + ndoc_w;  // total number of variables (with 6 dof per body)

    ndof = ncoords - ndoc;  // number of degrees of freedom (approximate - does not consider constr. redundancy, etc)

// BOOKKEEPING SANITY CHECK
// Test if the bookkeeping is properly aligned, at least for state gather/scatters,
// by filling a marked vector, and see if some gaps or overlaps are remaining.

#ifdef _DEBUG
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
        test_x.FillElem(poison_x);  // poison x
        test_v.FillElem(poison_v);  // poison v
        test_a.FillElem(poison_a);  // poison a
        test_L.FillElem(poison_L);  // poison L
        StateGather(test_x, test_v, test_T);
        StateGatherAcceleration(test_a);
        StateGatherReactions(test_L);
        for (int i = 0; i < test_x.GetRows(); ++i)
            assert(test_x(i) != poison_x);  // if your debugger breaks here, some ChPhysicsItem has a wrong
                                            // implementation of offsets or DOFs for positions
        for (int i = 0; i < test_v.GetRows(); ++i)
            assert(test_v(i) != poison_v);  // if your debugger breaks here, some ChPhysicsItem has a wrong
                                            // implementation of offsets or DOFs for velocities
        for (int i = 0; i < test_a.GetRows(); ++i)
            assert(test_a(i) != poison_a);  // if your debugger breaks here, some ChPhysicsItem has a wrong
                                            // implementation of offsets or DOFs for accelerations
        for (int i = 0; i < test_L.GetRows(); ++i)
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

void ChSystem::Update(bool update_assets) {
    timer_update.start();  // Timer for profiling

    // Executes the "forUpdate" in all controls of controlslist
    ExecuteControlsForUpdate();

    // Inherit parent class (recursively update sub objects bodies, links, etc)
    ChAssembly::Update(update_assets);

    // Update all contacts, if any
    contact_container->Update(ChTime, update_assets);

    timer_update.stop();
}

void ChSystem::IntStateGather(const unsigned int off_x,  // offset in x state vector
                              ChState& x,                // state vector, position part
                              const unsigned int off_v,  // offset in v state vector
                              ChStateDelta& v,           // state vector, speed part
                              double& T)                 // time
{
    unsigned int displ_x = off_x - offset_x;
    unsigned int displ_v = off_v - offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateGather(off_x, x, off_v, v, T);
    // Use also on contact container:
    contact_container->IntStateGather(displ_x + contact_container->GetOffset_x(), x,
                                      displ_v + contact_container->GetOffset_w(), v, T);
}

void ChSystem::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                               const ChState& x,          // state vector, position part
                               const unsigned int off_v,  // offset in v state vector
                               const ChStateDelta& v,     // state vector, speed part
                               const double T)            // time
{
    unsigned int displ_x = off_x - offset_x;
    unsigned int displ_v = off_v - offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateScatter(off_x, x, off_v, v, T);
    // Use also on contact container:
    contact_container->IntStateScatter(displ_x + contact_container->GetOffset_x(), x,
                                       displ_v + contact_container->GetOffset_w(), v, T);
}

void ChSystem::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    unsigned int displ_a = off_a - offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateGatherAcceleration(off_a, a);
    // Use also on contact container:
    contact_container->IntStateGatherAcceleration(displ_a + contact_container->GetOffset_w(), a);
}

/// From state derivative (acceleration) to system, sometimes might be needed
void ChSystem::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    unsigned int displ_a = off_a - offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateScatterAcceleration(off_a, a);
    // Use also on contact container:
    contact_container->IntStateScatterAcceleration(displ_a + contact_container->GetOffset_w(), a);
}

/// From system to reaction forces (last computed) - some timestepper might need this
void ChSystem::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    unsigned int displ_L = off_L - offset_L;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateGatherReactions(off_L, L);
    // Use also on contact container:
    contact_container->IntStateGatherReactions(displ_L + contact_container->GetOffset_L(), L);
}

/// From reaction forces to system, ex. store last computed reactions in ChLink objects for plotting etc.
void ChSystem::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    unsigned int displ_L = off_L - offset_L;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateScatterReactions(off_L, L);
    // Use also on contact container:
    contact_container->IntStateScatterReactions(displ_L + contact_container->GetOffset_L(), L);
}

void ChSystem::IntStateIncrement(const unsigned int off_x,  // offset in x state vector
                                 ChState& x_new,            // state vector, position part, incremented result
                                 const ChState& x,          // state vector, initial position part
                                 const unsigned int off_v,  // offset in v state vector
                                 const ChStateDelta& Dv)    // state vector, increment
{
    unsigned int displ_x = off_x - offset_x;
    unsigned int displ_v = off_v - offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateIncrement(off_x, x_new, x, off_v, Dv);
    // Use also on contact container:
    contact_container->IntStateIncrement(displ_x + contact_container->GetOffset_x(), x_new, x,
                                         displ_v + contact_container->GetOffset_w(), Dv);
}

void ChSystem::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                 ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                 const double c           // a scaling factor
                                 ) {
    unsigned int displ_v = off - offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntLoadResidual_F(off, R, c);
    // Use also on contact container:
    contact_container->IntLoadResidual_F(displ_v + contact_container->GetOffset_w(), R, c);
}

void ChSystem::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                  ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                  const ChVectorDynamic<>& w,  // the w vector
                                  const double c               // a scaling factor
                                  ) {
    unsigned int displ_v = off - offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntLoadResidual_Mv(off, R, w, c);
    // Use also on contact container:
    contact_container->IntLoadResidual_Mv(displ_v + contact_container->GetOffset_w(), R, w, c);
}

void ChSystem::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                   ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                   const ChVectorDynamic<>& L,  // the L vector
                                   const double c               // a scaling factor
                                   ) {
    unsigned int displ_L = off_L - offset_L;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntLoadResidual_CqL(off_L, R, L, c);
    // Use also on contact container:
    contact_container->IntLoadResidual_CqL(displ_L + contact_container->GetOffset_L(), R, L, c);
}

void ChSystem::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                   ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                   const double c,            // a scaling factor
                                   bool do_clamp,             // apply clamping to c*C?
                                   double recovery_clamp      // value for min/max clamping of c*C
                                   ) {
    unsigned int displ_L = off_L - offset_L;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntLoadConstraint_C(off_L, Qc, c, do_clamp, recovery_clamp);
    // Use also on contact container:
    contact_container->IntLoadConstraint_C(displ_L + contact_container->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
}

void ChSystem::IntLoadConstraint_Ct(const unsigned int off_L,  // offset in Qc residual
                                    ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*Ct
                                    const double c             // a scaling factor
                                    ) {
    unsigned int displ_L = off_L - offset_L;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntLoadConstraint_Ct(off_L, Qc, c);
    // Use also on contact container:
    contact_container->IntLoadConstraint_Ct(displ_L + contact_container->GetOffset_L(), Qc, c);
}

void ChSystem::IntToDescriptor(const unsigned int off_v,
                               const ChStateDelta& v,
                               const ChVectorDynamic<>& R,
                               const unsigned int off_L,
                               const ChVectorDynamic<>& L,
                               const ChVectorDynamic<>& Qc) {
    unsigned int displ_L = off_L - offset_L;
    unsigned int displ_v = off_v - offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntToDescriptor(off_v, v, R, off_L, L, Qc);
    // Use also on contact container:
    contact_container->IntToDescriptor(displ_v + contact_container->GetOffset_w(), v, R,
                                       displ_L + contact_container->GetOffset_L(), L, Qc);
}

void ChSystem::IntFromDescriptor(const unsigned int off_v,
                                 ChStateDelta& v,
                                 const unsigned int off_L,
                                 ChVectorDynamic<>& L) {
    unsigned int displ_L = off_L - offset_L;
    unsigned int displ_v = off_v - offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntFromDescriptor(off_v, v, off_L, L);
    // Use also on contact container:
    contact_container->IntFromDescriptor(displ_v + contact_container->GetOffset_w(), v,
                                         displ_L + contact_container->GetOffset_L(), L);
}

// -----------------------------------------------------------------------------

void ChSystem::InjectVariables(ChSystemDescriptor& mdescriptor) {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::InjectVariables(mdescriptor);
    // Use also on contact container:
    contact_container->InjectVariables(mdescriptor);
}

void ChSystem::VariablesFbReset() {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::VariablesFbReset();
    // Use also on contact container:
    contact_container->VariablesFbReset();
}

void ChSystem::VariablesFbLoadForces(double factor) {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::VariablesFbLoadForces();
    // Use also on contact container:
    contact_container->VariablesFbLoadForces();
}

void ChSystem::VariablesFbIncrementMq() {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::VariablesFbIncrementMq();
    // Use also on contact container:
    contact_container->VariablesFbIncrementMq();
}

void ChSystem::VariablesQbLoadSpeed() {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::VariablesQbLoadSpeed();
    // Use also on contact container:
    contact_container->VariablesQbLoadSpeed();
}

void ChSystem::VariablesQbSetSpeed(double step) {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::VariablesQbSetSpeed(step);
    // Use also on contact container:
    contact_container->VariablesQbSetSpeed(step);
}

void ChSystem::VariablesQbIncrementPosition(double dt_step) {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::VariablesQbIncrementPosition(dt_step);
    // Use also on contact container:
    contact_container->VariablesQbIncrementPosition(dt_step);
}

void ChSystem::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::InjectConstraints(mdescriptor);
    // Use also on contact container:
    contact_container->InjectConstraints(mdescriptor);
}

void ChSystem::ConstraintsBiReset() {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::ConstraintsBiReset();
    // Use also on contact container:
    contact_container->ConstraintsBiReset();
}

void ChSystem::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
    // Use also on contact container:
    contact_container->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
}

void ChSystem::ConstraintsBiLoad_Ct(double factor) {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::ConstraintsBiLoad_Ct(factor);
    // Use also on contact container:
    contact_container->ConstraintsBiLoad_Ct(factor);
}

void ChSystem::ConstraintsBiLoad_Qc(double factor) {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::ConstraintsBiLoad_Qc(factor);
    // Use also on contact container:
    contact_container->ConstraintsBiLoad_Qc(factor);
}

void ChSystem::ConstraintsFbLoadForces(double factor) {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::ConstraintsFbLoadForces(factor);
    // Use also on contact container:
    contact_container->ConstraintsFbLoadForces(factor);
}

void ChSystem::ConstraintsLoadJacobians() {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::ConstraintsLoadJacobians();
    // Use also on contact container:
    contact_container->ConstraintsLoadJacobians();
}

void ChSystem::ConstraintsFetch_react(double factor) {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::ConstraintsFetch_react(factor);
    // Use also on contact container:
    contact_container->ConstraintsFetch_react(factor);
}

void ChSystem::InjectKRMmatrices(ChSystemDescriptor& mdescriptor) {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::InjectKRMmatrices(mdescriptor);
    // Use also on contact container:
    contact_container->InjectKRMmatrices(mdescriptor);
}

void ChSystem::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
    // Use also on contact container:
    contact_container->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
}

// -----------------------------------------------------------------------------
//    TIMESTEPPER INTERFACE
// -----------------------------------------------------------------------------

// From system to state y={x,v}
void ChSystem::StateGather(ChState& x, ChStateDelta& v, double& T) {
    IntStateGather(0, x, 0, v, T);
}

// From state Y={x,v} to system.
void ChSystem::StateScatter(const ChState& x, const ChStateDelta& v, const double T) {
    IntStateScatter(0, x, 0, v, T);

    Update();  //***TODO*** optimize because maybe IntStateScatter above might have already called Update?
}

// From system to state derivative (acceleration), some timesteppers might need last computed accel.
void ChSystem::StateGatherAcceleration(ChStateDelta& a) {
    IntStateGatherAcceleration(0, a);
}

// From state derivative (acceleration) to system, sometimes might be needed
void ChSystem::StateScatterAcceleration(const ChStateDelta& a) {
    IntStateScatterAcceleration(0, a);
}

// From system to reaction forces (last computed) - some timestepper might need this
void ChSystem::StateGatherReactions(ChVectorDynamic<>& L) {
    IntStateGatherReactions(0, L);
}

// From reaction forces to system, ex. store last computed reactions in ChLink objects for plotting etc.
void ChSystem::StateScatterReactions(const ChVectorDynamic<>& L) {
    IntStateScatterReactions(0, L);
}

// Perform x_new = x + dx    for x in    Y = {x, dx/dt}
// It takes care of the fact that x has quaternions, dx has angular vel etc.
// NOTE: the system is not updated automatically after the state increment, so one might
// need to call StateScatter() if needed.
void ChSystem::StateIncrementX(ChState& x_new,         ///< resulting x_new = x + Dx
                               const ChState& x,       ///< initial state x
                               const ChStateDelta& Dx  ///< state increment Dx
                               ) {
    IntStateIncrement(0, x_new, x, 0, Dx);
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
                                    const double c_x,             // the factor in c_x*dF/dv
                                    const ChState& x,             // current state, x part
                                    const ChStateDelta& v,        // current state, v part
                                    const double T,               // current time T
                                    bool force_state_scatter,     // if false, x,v and T are not scattered to the system
                                    bool force_setup              // if true, call the solver's Setup() function
                                    ) {
    if (force_state_scatter)
        StateScatter(x, v, T);

    // R and Qc vectors  --> solver sparse solver structures  (also sets L and Dv to warmstart)
    IntToDescriptor(0, Dv, R, 0, L, Qc);

    // If the solver's Setup() must be called or if the solver's Solve() requires it,
    // fill the sparse system structures with information in G and Cq.
    if (force_setup || GetSolver()->SolveRequiresMatrix()) {
        // Cq  matrix
        ConstraintsLoadJacobians();

        // G matrix: M, K, R components
        if (c_a || c_v || c_x)
            KRMmatricesLoad(-c_x, -c_v, c_a);

        // For ChVariable objects without a ChKblock, just use the 'a' coefficient
        descriptor->SetMassFactor(c_a);
    }

    // Diagnostics:
    if (dump_matrices) {

        const char* numformat = "%.12g";
        std::string sprefix = "solve_" + std::to_string(stepcount) + "_" + std::to_string(solvecount) + "_";

        descriptor->DumpLastMatrices(true, sprefix.c_str());
        descriptor->DumpLastMatrices(false, sprefix.c_str());

        chrono::ChStreamOutAsciiFile file_x((sprefix + "x_pre.dat").c_str());
        file_x.SetNumFormat(numformat);
        ((ChMatrix<>)x).StreamOUTdenseMatlabFormat(file_x);

        chrono::ChStreamOutAsciiFile file_v((sprefix + "v_pre.dat").c_str());
        file_v.SetNumFormat(numformat);
        ((ChMatrix<>)v).StreamOUTdenseMatlabFormat(file_v);
    }

    // If indicated, first perform a solver setup.
    // Return 'false' if the setup phase fails.
    if (force_setup) {
        timer_setup.start();
        bool success = GetSolver()->Setup(*descriptor);
        timer_setup.stop();
        setupcount++;
        if (!success)
            return false;
    }

    // Solve the problem
    // The solution is scattered in the provided system descriptor
    timer_solver.start();
    GetSolver()->Solve(*descriptor);
    timer_solver.stop();
    

    // Dv and L vectors  <-- sparse solver structures
    IntFromDescriptor(0, Dv, 0, L);

    // Diagnostics:
    if (dump_matrices) {
        const char* numformat = "%.12g";
        std::string sprefix = "solve_" + std::to_string(stepcount) + "_" + std::to_string(solvecount) + "_";

        chrono::ChStreamOutAsciiFile file_Dv((sprefix + "Dv.dat").c_str());
        file_Dv.SetNumFormat(numformat);
        ((ChMatrix<>)Dv).StreamOUTdenseMatlabFormat(file_Dv);

        chrono::ChStreamOutAsciiFile file_L((sprefix + "L.dat").c_str());
        file_L.SetNumFormat(numformat);
        ((ChMatrix<>)L).StreamOUTdenseMatlabFormat(file_L);

        // Just for diagnostic, dump also unscaled loads (forces,torques), 
        // since the .._f.dat vector dumped in DumpLastMatrices() might contain scaled loads, and also +M*v
        ChVectorDynamic<> tempF(this->GetNcoords_v());
        this->IntLoadResidual_F(0, tempF, 1.0);
        chrono::ChStreamOutAsciiFile file_F((sprefix + "F_pre.dat").c_str());
        file_F.SetNumFormat(numformat);
        tempF.StreamOUTdenseMatlabFormat(file_F);
    }
    
    solvecount++;

    return true;
}

// Increment a vector R with the term c*F:
//    R += c*F
void ChSystem::LoadResidual_F(ChVectorDynamic<>& R,  ///< result: the R residual, R += c*F
                              const double c         ///< a scaling factor
                              ) {
    IntLoadResidual_F(0, R, c);
}

// Increment a vector R with a term that has M multiplied a given vector w:
//    R += c*M*w
void ChSystem::LoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                               const ChVectorDynamic<>& w,  ///< the w vector
                               const double c               ///< a scaling factor
                               ) {
    IntLoadResidual_Mv(0, R, w, c);
}

// Increment a vectorR with the term Cq'*L:
//    R += c*Cq'*L
void ChSystem::LoadResidual_CqL(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                const ChVectorDynamic<>& L,  ///< the L vector
                                const double c               ///< a scaling factor
                                ) {
    IntLoadResidual_CqL(0, R, L, c);
}

// Increment a vector Qc with the term C:
//    Qc += c*C
void ChSystem::LoadConstraint_C(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*C
                                const double c,         ///< a scaling factor
                                const bool mdo_clamp,   ///< enable optional clamping of Qc
                                const double mclam      ///< clamping value
                                ) {
    IntLoadConstraint_C(0, Qc, c, mdo_clamp, mclam);
}

// Increment a vector Qc with the term Ct = partial derivative dC/dt:
//    Qc += c*Ct
void ChSystem::LoadConstraint_Ct(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*Ct
                                 const double c          ///< a scaling factor
                                 ) {
    IntLoadConstraint_Ct(0, Qc, c);
}
// -----------------------------------------------------------------------------
//   COLLISION OPERATIONS
// -----------------------------------------------------------------------------

int ChSystem::GetNcontacts() {
    return contact_container->GetNcontacts();
}

void ChSystem::SynchronizeLastCollPositions() {
    for (int ip = 0; ip < bodylist.size(); ++ip) {
        if (bodylist[ip]->GetCollide())
            bodylist[ip]->SynchronizeLastCollPos();
    }
}

class SystemAddCollisionPointCallback : public ChAddContactCallback {
  public:
    ChSystem* client_system;
    virtual void ContactCallback(
        const collision::ChCollisionInfo& mcontactinfo,  ///< pass info about contact (cannot change it)
        ChMaterialCouple& material                       ///< you can modify this!
        ) {
        if (client_system->collisionpoint_callback)
            client_system->collisionpoint_callback->ContactCallback(mcontactinfo, material);
    }
};

double ChSystem::ComputeCollisions() {
    double mretC = 0.0;

    timer_collision_broad.start();

    // Update all positions of collision models: delegate this to the ChAssembly
    SyncCollisionModels();

    // Prepare the callback

    // In case there is some user callback for each added point..
    SystemAddCollisionPointCallback mpointcallback;
    if (collisionpoint_callback) {
        mpointcallback.client_system = this;
        contact_container->SetAddContactCallback(&mpointcallback);
    } else {
        contact_container->SetAddContactCallback(0);
    }

    // !!! Perform the collision detection ( broadphase and narrowphase ) !!!

    collision_system->Run();

    // Report and store contacts and/or proximities, if there are some
    // containers in the physic system. The default contact container
    // for ChBody and ChParticles is used always.

    collision_system->ReportContacts(contact_container.get());

    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        if (auto mcontactcontainer = std::dynamic_pointer_cast<ChContactContainerBase>(otherphysicslist[ip])) {
            collision_system->ReportContacts(mcontactcontainer.get());
        }

        if (auto mproximitycontainer = std::dynamic_pointer_cast<ChProximityContainerBase>(otherphysicslist[ip])) {
            collision_system->ReportProximities(mproximitycontainer.get());
        }
    }

    // If some other collision engine could add further ChLinkContact into the list..
    for (size_t ic = 0; ic < collision_callbacks.size(); ic++)
        collision_callbacks[ic]->PerformCustomCollision(this);

    // Count the contacts of body-body type.
    ncontacts = contact_container->GetNcontacts();

    timer_collision_broad.stop();

    return mretC;
}

// =============================================================================
//   PHYSICAL OPERATIONS
// =============================================================================


void ChSystem::GetMassMatrix(ChSparseMatrix* M) {
    //IntToDescriptor(0, Dv, R, 0, L, Qc);
    //ConstraintsLoadJacobians();
    
        // Load all KRM matrices with the M part only
    KRMmatricesLoad(0, 0, 1.0); 
        // For ChVariable objects without a ChKblock, but still with a mass:
    descriptor->SetMassFactor(1.0);

        // Fill system-level M matrix
    this->GetSystemDescriptor()->ConvertToMatrixForm(nullptr, M, nullptr, nullptr, nullptr, nullptr, false, false);
}

void ChSystem::GetStiffnessMatrix(ChSparseMatrix* K) {
    //IntToDescriptor(0, Dv, R, 0, L, Qc);
    //ConstraintsLoadJacobians();
    
        // Load all KRM matrices with the K part only
    this->KRMmatricesLoad(1.0, 0, 0); 
        // For ChVariable objects without a ChKblock, but still with a mass:
    descriptor->SetMassFactor(0.0);

        // Fill system-level K matrix
    this->GetSystemDescriptor()->ConvertToMatrixForm(nullptr, K, nullptr, nullptr, nullptr, nullptr, false, false);
}

void ChSystem::GetDampingMatrix(ChSparseMatrix* R) {
    //IntToDescriptor(0, Dv, R, 0, L, Qc);
    //ConstraintsLoadJacobians();
    
        // Load all KRM matrices with the R part only
    this->KRMmatricesLoad(0, 1.0, 0); 
        // For ChVariable objects without a ChKblock, but still with a mass:
    descriptor->SetMassFactor(0.0);

        // Fill system-level R matrix
    this->GetSystemDescriptor()->ConvertToMatrixForm(nullptr, R, nullptr, nullptr, nullptr, nullptr, false, false);
}

void ChSystem::GetConstraintJacobianMatrix(ChSparseMatrix* Cq) {
    //IntToDescriptor(0, Dv, R, 0, L, Qc);

        // Load all jacobian matrices 
    this->ConstraintsLoadJacobians();

        // Fill system-level R matrix
    this->GetSystemDescriptor()->ConvertToMatrixForm(Cq, nullptr, nullptr, nullptr, nullptr, nullptr, false, false);
}

void ChSystem::DumpSystemMatrices(bool save_M, bool save_K, bool save_R, bool save_Cq, const char* path) {
    char filename[300];
    const char* numformat = "%.12g";

    if (save_M) {
        ChLinkedListMatrix mM;
        this->GetMassMatrix(&mM);
        sprintf(filename, "%s%s", path, "_M.dat");
        ChStreamOutAsciiFile file_M(filename);
        file_M.SetNumFormat(numformat);
        mM.StreamOUTsparseMatlabFormat(file_M);
    }
    if (save_K) {
        ChLinkedListMatrix mK;
        this->GetStiffnessMatrix(&mK);
        sprintf(filename, "%s%s", path, "_K.dat");
        ChStreamOutAsciiFile file_K(filename);
        file_K.SetNumFormat(numformat);
        mK.StreamOUTsparseMatlabFormat(file_K);
    }
    if (save_R) {
        ChLinkedListMatrix mR;
        this->GetDampingMatrix(&mR);
        sprintf(filename, "%s%s", path, "_R.dat");
        ChStreamOutAsciiFile file_R(filename);
        file_R.SetNumFormat(numformat);
        mR.StreamOUTsparseMatlabFormat(file_R);
    }
    if (save_Cq) {
        ChLinkedListMatrix mCq;
        this->GetConstraintJacobianMatrix(&mCq);
        sprintf(filename, "%s%s", path, "_Cq.dat");
        ChStreamOutAsciiFile file_Cq(filename);
        file_Cq.SetNumFormat(numformat);
        mCq.StreamOUTsparseMatlabFormat(file_Cq);
    }  
}





// -----------------------------------------------------------------------------
//  PERFORM AN INTEGRATION STEP.  ----
//
//  Advances a single time step.
//
//  Note that time step can be modified if some variable-time stepper is used.
// -----------------------------------------------------------------------------

int ChSystem::DoStepDynamics(double m_step) {
    step = m_step;
    return Integrate_Y();
}

// -----------------------------------------------------------------------------
//  PERFORM INTEGRATION STEP  using pluggable timestepper
// -----------------------------------------------------------------------------

bool ChSystem::Integrate_Y() {
    ResetTimers();

    timer_step.start();

    // Executes "forStep" in all controls of controlslist
    ExecuteControlsForStep();

    stepcount++;
    solvecount = 0;
    setupcount = 0;

    // Compute contacts and create contact constraints
    ComputeCollisions();

    // Counts dofs, statistics, etc. (not needed because already in Advance()...? )
    Setup();

    // Update everything - and put to sleep bodies that need it (not needed because already in Advance()...? )
    // No need to update visualization assets here.
    Update(false);

    // Re-wake the bodies that cannot sleep because they are in contact with
    // some body that is not in sleep state.
    ManageSleepingBodies();

    // Prepare lists of variables and constraints.
    DescriptorPrepareInject(*descriptor);
    descriptor->UpdateCountsAndOffsets();

    // Set some settings in timestepper object
    timestepper->SetQcDoClamp(true);
    timestepper->SetQcClamping(max_penetration_recovery_speed);
    if (std::dynamic_pointer_cast<ChTimestepperHHT>(timestepper) ||
        std::dynamic_pointer_cast<ChTimestepperNewmark>(timestepper))
        timestepper->SetQcDoClamp(false);

    // PERFORM TIME STEP HERE!
    timestepper->Advance(step);

    // Executes custom processing at the end of step
    CustomEndOfStep();

    // If there are some probe objects in the probe list,
    // tell them to record their variables (ususally x-y couples)
    RecordAllProbes();

    // Call method to gather contact forces/torques in rigid bodies
    contact_container->ComputeContactForces();

    // Time elapsed for step..
    timer_step.stop();

    return true;
}

// -----------------------------------------------------------------------------
// **** SATISFY ALL COSTRAINT EQUATIONS WITH NEWTON
// **** ITERATION, UNTIL TOLERANCE SATISFIED, THEN UPDATE
// **** THE "Y" STATE WITH SetY (WHICH AUTOMATICALLY UPDATES
// **** ALSO AUXILIARY MATRICES).
// -----------------------------------------------------------------------------

bool ChSystem::DoAssembly(int action) {
    solvecount = 0;
    setupcount = 0;

    Setup();
    Update();

    int old_maxsteps = GetMaxItersSolverSpeed();
    SetMaxItersSolverSpeed(300);

    double old_step = GetStep();
    double new_step = 1e-6;
    SetStep(new_step);

    double old_tol = GetTolForce();
    SetTolForce(1e-4);

    // Prepare lists of variables and constraints.
    DescriptorPrepareInject(*descriptor);

    ChAssemblyAnalysis manalysis(*this);
    manalysis.SetMaxAssemblyIters(GetMaxiter());

    // Perform analysis
    manalysis.AssemblyAnalysis(action, new_step);

    SetMaxItersSolverSpeed(old_maxsteps);
    SetStep(old_step);
    SetTolForce(old_tol);

    return true;
}

// -----------------------------------------------------------------------------
// **** PERFORM THE LINEAR STATIC ANALYSIS
// -----------------------------------------------------------------------------

bool ChSystem::DoStaticLinear() {
    solvecount = 0;
    setupcount = 0;

    Setup();
    Update();

    int old_maxsteps = GetMaxItersSolverSpeed();
    SetMaxItersSolverSpeed(300);

    // Prepare lists of variables and constraints.
    DescriptorPrepareInject(*descriptor);

    ChStaticLinearAnalysis manalysis(*this);

    // Perform analysis
    manalysis.StaticAnalysis();

    SetMaxItersSolverSpeed(old_maxsteps);

    bool dump_data = false;

    if (dump_data) {
        GetSystemDescriptor()->DumpLastMatrices();

        // optional check for correctness in result
        chrono::ChMatrixDynamic<double> md;
        GetSystemDescriptor()->BuildDiVector(md);  // d={f;-b}

        chrono::ChMatrixDynamic<double> mx;
        GetSystemDescriptor()->FromUnknownsToVector(mx);  // x ={q,-l}
        chrono::ChStreamOutAsciiFile file_x("dump_x.dat");
        mx.StreamOUTdenseMatlabFormat(file_x);

        chrono::ChMatrixDynamic<double> mZx;
        GetSystemDescriptor()->SystemProduct(mZx, &mx);  // Zx = Z*x

        GetLog() << "CHECK: norm of solver residual: ||Z*x-d|| -------------------\n";
        GetLog() << (mZx - md).NormInf() << "\n";
    }

    return true;
}

// -----------------------------------------------------------------------------
// **** PERFORM THE NONLINEAR STATIC ANALYSIS
// -----------------------------------------------------------------------------

bool ChSystem::DoStaticNonlinear(int nsteps) {
    solvecount = 0;
    setupcount = 0;

    Setup();
    Update();

    int old_maxsteps = GetMaxItersSolverSpeed();
    SetMaxItersSolverSpeed(300);

    // Prepare lists of variables and constraints.
    DescriptorPrepareInject(*descriptor);

    ChStaticNonLinearAnalysis manalysis(*this);
    manalysis.SetMaxiters(nsteps);

    // Perform analysis
    manalysis.StaticAnalysis();

    SetMaxItersSolverSpeed(old_maxsteps);

    return true;
}

// -----------------------------------------------------------------------------
// **** PERFORM THE STATIC ANALYSIS, FINDING THE STATIC
// **** EQUILIBRIUM OF THE SYSTEM, WITH ITERATIVE SOLUTION
// -----------------------------------------------------------------------------

bool ChSystem::DoStaticRelaxing(int nsteps) {
    solvecount = 0;
    setupcount = 0;

    int err = 0;
    bool reached_tolerance = false;

    if ((ncoords > 0) && (ndof >= 0)) {
        for (int m_iter = 0; m_iter < nsteps; m_iter++) {
            for (int ip = 0; ip < bodylist.size(); ++ip) {
                // Set no body speed and no body accel.
                bodylist[ip]->SetNoSpeedNoAcceleration();
            }
            for (int ip = 0; ip < otherphysicslist.size(); ++ip) {
                otherphysicslist[ip]->SetNoSpeedNoAcceleration();
            }

            double m_undotime = GetChTime();
            DoFrameDynamics(m_undotime + (step * 1.8) * (((double)nsteps - (double)m_iter)) / (double)nsteps);
            SetChTime(m_undotime);
        }

        for (int ip = 0; ip < bodylist.size(); ++ip) {
            // Set no body speed and no body accel.
            bodylist[ip]->SetNoSpeedNoAcceleration();
        }

        for (int ip = 0; ip < otherphysicslist.size(); ++ip) {
            otherphysicslist[ip]->SetNoSpeedNoAcceleration();
        }
    }

    if (err) {
        last_err = true;
        GetLog() << "WARNING: some costraints may be redundant, but couldn't be eliminated \n";
    }
    return last_err;
}

// -----------------------------------------------------------------------------
// **** ---    THE KINEMATIC SIMULATION  ---
// **** PERFORM IK (INVERSE KINEMATICS) UNTIL THE END_TIME IS
// **** REACHED, STARTING FROM THE CURRENT TIME.
// -----------------------------------------------------------------------------

bool ChSystem::DoEntireKinematics() {
    Setup();

    int action = AssemblyLevel::POSITION | AssemblyLevel::VELOCITY | AssemblyLevel::ACCELERATION;

    DoAssembly(action);
    // first check if there are redundant links (at least one NR cycle
    // even if the structure is already assembled)

    while (ChTime < end_time) {
        // Newton-Raphson iteration, closing constraints
        DoAssembly(action);

        if (last_err)
            return false;

        // Update time and repeat.
        ChTime += step;
    }

    return true;
}

// -----------------------------------------------------------------------------
// **** ---   THE DYNAMICAL SIMULATION   ---
// **** PERFORM EXPLICIT OR IMPLICIT INTEGRATION TO GET
// **** THE DYNAMICAL SIMULATION OF THE SYSTEM, UNTIL THE
// **** END_TIME IS REACHED.
// -----------------------------------------------------------------------------

bool ChSystem::DoEntireDynamics() {
    Setup();

    // the system may have wrong layout, or too large
    // clearances in costraints, so it is better to
    // check for costraint violation each time the integration starts
    DoAssembly(AssemblyLevel::POSITION | AssemblyLevel::VELOCITY | AssemblyLevel::ACCELERATION);

    // Perform the integration steps until the end
    // time is reached.
    // All the updating (of Y, Y_dt and time) is done
    // automatically by Integrate()

    while (ChTime < end_time) {
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
// the specified m_endtime, and terminating the integration exactly
// on the m_endtime. Therefore, the step of integration may get a
// little increment/decrement to have the last step ending in m_endtime.
// Note that this function can be used in iterations to provide results in
// a evenly spaced frames of time, even if the steps are changing.
// Also note that if the time step is higher than the time increment
// requested to reach m_endtime, the step is lowered.

bool ChSystem::DoFrameDynamics(double m_endtime) {
    double frame_step;
    double old_step;
    double left_time;
    bool restore_oldstep = false;
    int counter = 0;
    double fixed_step_undo;

    frame_step = (m_endtime - ChTime);
    fixed_step_undo = step;

    while (ChTime < m_endtime) {
        restore_oldstep = false;
        counter++;

        left_time = m_endtime - ChTime;

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

bool ChSystem::DoEntireUniformDynamics(double frame_step) {
    // the initial system may have wrong layout, or too large
    // clearances in constraints.
    Setup();
    DoAssembly(AssemblyLevel::POSITION | AssemblyLevel::VELOCITY | AssemblyLevel::ACCELERATION);

    while (ChTime < end_time) {
        double goto_time = (ChTime + frame_step);
        if (!DoFrameDynamics(goto_time))
            return false;
    }

    return true;
}

// Like DoFrameDynamics, but performs kinematics instead of dinamics

bool ChSystem::DoFrameKinematics(double m_endtime) {
    double frame_step;
    double old_step;
    double left_time;
    int restore_oldstep;
    int counter = 0;

    frame_step = (m_endtime - ChTime);

    double fixed_step_undo = step;

    while (ChTime < m_endtime) {
        restore_oldstep = false;
        counter++;

        left_time = m_endtime - ChTime;

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

        ChTime += step;

        if (restore_oldstep)
            step = old_step;  // if timestep was changed to meet the end of frametime
    }

    return true;
}

bool ChSystem::DoStepKinematics(double m_step) {
    ChTime += m_step;

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

    // serialize parent class
    ChAssembly::ArchiveOUT(marchive);

    // serialize all member data:

    marchive << CHNVP(contact_container);

    marchive << CHNVP(G_acc);
    marchive << CHNVP(end_time);
    marchive << CHNVP(step);
    marchive << CHNVP(step_min);
    marchive << CHNVP(step_max);
    marchive << CHNVP(stepcount);
    marchive << CHNVP(dump_matrices);

    marchive << CHNVP(tol);
    marchive << CHNVP(tol_force);
    marchive << CHNVP(maxiter);
    marchive << CHNVP(use_sleeping);

    marchive << CHNVP(descriptor);
    marchive << CHNVP(solver_speed);
    marchive << CHNVP(solver_stab);

    marchive << CHNVP(max_iter_solver_speed);
    marchive << CHNVP(max_iter_solver_stab);
    marchive << CHNVP(max_steps_simplex);
    marchive << CHNVP(min_bounce_speed);
    marchive << CHNVP(max_penetration_recovery_speed);
    marchive << CHNVP(parallel_thread_number);

    marchive << CHNVP(collision_system);  // ChCollisionSystem should implement class factory for abstract create

    marchive << CHNVP(timestepper);  // ChTimestepper should implement class factory for abstract create

    //***TODO*** complete...
}

// Method to allow de serialization of transient data from archives.
void ChSystem::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChSystem>();

    // deserialize parent class
    ChAssembly::ArchiveIN(marchive);

    // stream in all member data:

    marchive >> CHNVP(contact_container);

    marchive >> CHNVP(G_acc);
    marchive >> CHNVP(end_time);
    marchive >> CHNVP(step);
    marchive >> CHNVP(step_min);
    marchive >> CHNVP(step_max);
    marchive >> CHNVP(stepcount);
    marchive >> CHNVP(dump_matrices);

    marchive >> CHNVP(tol);
    marchive >> CHNVP(tol_force);
    marchive >> CHNVP(maxiter);
    marchive >> CHNVP(use_sleeping);

    marchive >> CHNVP(descriptor);
    marchive >> CHNVP(solver_speed);
    marchive >> CHNVP(solver_stab);

    marchive >> CHNVP(max_iter_solver_speed);
    marchive >> CHNVP(max_iter_solver_stab);
    marchive >> CHNVP(max_steps_simplex);
    marchive >> CHNVP(min_bounce_speed);
    marchive >> CHNVP(max_penetration_recovery_speed);
    marchive >> CHNVP(parallel_thread_number);

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
