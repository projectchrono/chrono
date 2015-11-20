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


#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>
#include <algorithm>

#include "physics/ChSystem.h"
#include "physics/ChGlobal.h"
#include "physics/ChBodyAuxRef.h"
#include "physics/ChContactContainerDVI.h"
#include "physics/ChProximityContainerBase.h"

#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpSimplexSolver.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativeSymmSOR.h"
#include "lcp/ChLcpIterativeSORmultithread.h"
#include "lcp/ChLcpIterativeJacobi.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "lcp/ChLcpIterativeBB.h"
#include "lcp/ChLcpIterativePCG.h"
#include "lcp/ChLcpIterativeAPGD.h"
#include "parallel/ChOpenMP.h"

#include "core/ChTimer.h"
#include "collision/ChCCollisionSystemBullet.h"
#include "collision/ChCModelBullet.h"
#include "timestepper/ChTimestepper.h"
#include "timestepper/ChStaticAnalysis.h"

using namespace chrono::collision;

namespace chrono {







/// Class for iterating through all items of ChPhysicalItem that exist in
/// a ChSystem.
/// It will iterate through:
/// - all ChBody          objects
/// - all ChLink          objects
/// - all ChPhysicalItems objects that were added, that were not ChBody or ChLink
/// - the ChContactContainer object that manages all the contacts.
/// Note that this iterator suffers a small overhead if compared to the use of
/// three iterator cycles in succession (one over ChBody list, one over ChLink list, etc.)
/// but it makes the code much more readable. Use it as follows:
///    IteratorAllPhysics my_iter(this);
///    while(my_iter.HasItem())
///    {
///        my_iter->....
///        ++my_iter;
///    }

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
        mptr = ChSharedPtr<ChPhysicsItem>(0);
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
                    mptr = ChSharedPtr<ChPhysicsItem>(0);
                    return (*this);
                }
            }  // end cases
        } while (true);

        return (*this);
    }

    ChSharedPtr<ChPhysicsItem> operator->() { return (mptr); }
    ChSharedPtr<ChPhysicsItem> operator*() { return (mptr); }

  private:
    std::vector< ChSharedPtr<ChBody> >::iterator node_body;
    std::vector< ChSharedPtr<ChBody> >* list_bodies;
    std::vector< ChSharedPtr<ChLink> >::iterator node_link;
    std::vector< ChSharedPtr<ChLink> >* list_links;
    std::vector< ChSharedPtr<ChPhysicsItem> >::iterator node_otherphysics;
    std::vector< ChSharedPtr<ChPhysicsItem> >* list_otherphysics;
    ChSharedPtr<ChPhysicsItem> mptr;
    int stage;
    ChSystem* msystem;
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CLASS FOR PHYSICAL SYSTEM

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChSystem> a_registration_ChSystem;

ChSystem::ChSystem(unsigned int max_objects, double scene_size, bool init_sys) {

    this->system = this; // as needed by ChAssembly

    probelist.clear();
    controlslist.clear();

    // ------ PREFERENCES INITIALIZATION :
    ChTime = 0;

    end_time = 1;
    step = 0.04;
    step_min = 0.002;
    step_max = 0.04;
    tol = 0.0002;
    tol_force = 1e-3;
    maxiter = 6;

    ncontacts = 0;

    SetIntegrationType(INT_EULER_IMPLICIT_LINEARIZED); 

    min_bounce_speed = 0.15;
    max_penetration_recovery_speed = 0.6;

    parallel_thread_number = CHOMPfunctions::GetNumProcs();  // default n.threads as n.cores

    //this->contact_container = 0;
    // default contact container
    if (init_sys) {
        this->contact_container = ChSharedPtr<ChContactContainerDVI>(new ChContactContainerDVI());
        this->contact_container->SetSystem(this);
    }
    collision_system = 0;
    // default collision engine
    if (init_sys) {
        collision_system = new ChCollisionSystemBullet(max_objects, scene_size);
    }

    this->timestepper =
        ChSharedPtr<ChTimestepperEulerImplicitLinearized>(new ChTimestepperEulerImplicitLinearized(this));  // OK

    collisionpoint_callback = 0;

    LCP_descriptor = 0;
    LCP_solver_speed = 0;
    LCP_solver_stab = 0;

    iterLCPmaxIters = 30;
    iterLCPmaxItersStab = 10;
    
    if (init_sys) {
        SetLcpSolverType(LCP_ITERATIVE_SYMMSOR);
    }

    use_sleeping = false;

    collisionpoint_callback = 0;

    Set_G_acc(ChVector<>(0, -9.8, 0));

    stepcount = 0;
    solvecount = 0;
    dump_matrices = false;

    last_err = 0;

    scriptEngine = 0;
    scriptForStart = NULL;
    scriptForUpdate = NULL;
    scriptForStep = NULL;
    scriptFor3DStep = NULL;

    events = new ChEvents(250);
}

ChSystem::~ChSystem() {

    // Before proceeding, anticipate Clear(). This would be called also by base ChAssembly destructor, anyway, but
    // it would happen after this destructor, so the ith_body->SetSystem(0) in Clear() would not be able to remove body's collision
    // models from the collision_system. Here it is possible, since the collision_system is still alive.
    Clear();

    RemoveAllProbes();
    RemoveAllControls();

    if (LCP_solver_speed)
        delete LCP_solver_speed;
    LCP_solver_speed = 0;
    if (LCP_solver_stab)
        delete LCP_solver_stab;
    LCP_solver_stab = 0;
    if (LCP_descriptor)
        delete LCP_descriptor;
    LCP_descriptor = 0;

    if (collision_system)
        delete collision_system;
    collision_system = 0;

    if (events)
        delete events;
    events = 0;

    if (scriptForStart)
        delete scriptForStart;
    if (scriptForUpdate)
        delete scriptForUpdate;
    if (scriptForStep)
        delete scriptForStep;
    if (scriptFor3DStep)
        delete scriptFor3DStep;
}

void ChSystem::Copy(ChSystem* source) {
    // first the parent class data...
    ChAssembly::Copy(source);

    this->system = this; // as needed by ChAssembly

    G_acc = source->Get_G_acc();
    end_time = source->GetEndTime();
    step = source->GetStep();
    step_min = source->GetStepMin();
    step_max = source->GetStepMax();
    stepcount = source->stepcount;
    solvecount = source->solvecount;
    dump_matrices = source->dump_matrices;
    SetIntegrationType(source->GetIntegrationType());
    tol = source->GetTol();
    tol_force = source->tol_force;
    maxiter = source->GetMaxiter();
    
    min_bounce_speed = source->min_bounce_speed;
    max_penetration_recovery_speed = source->max_penetration_recovery_speed;
    iterLCPmaxIters = source->iterLCPmaxIters;
    iterLCPmaxItersStab = source->iterLCPmaxItersStab;
    SetLcpSolverType(GetLcpSolverType());
    parallel_thread_number = source->parallel_thread_number;
    use_sleeping = source->use_sleeping;

    ncontacts = source->ncontacts;

    collision_callbacks = source->collision_callbacks;
    collisionpoint_callback = source->collisionpoint_callback;

    last_err = source->last_err;

    RemoveAllProbes();
    RemoveAllControls();

    events->ResetAllEvents();  // don't copy events.

    SetScriptForStartFile(source->scriptForStartFile);
    SetScriptForUpdateFile(source->scriptForUpdateFile);
    SetScriptForStepFile(source->scriptForStepFile);
    SetScriptFor3DStepFile(source->scriptFor3DStepFile);

    ChTime = source->ChTime;
}

void ChSystem::Clear() {
    // first the parent class data...
    ChAssembly::Clear();


    events->ResetAllEvents();

    // contact_container->RemoveAllContacts();

    RemoveAllProbes();
    RemoveAllControls();

    // ResetTimers();
}

//
// Set/Get routines
//

void ChSystem::SetLcpSolverType(eCh_lcpSolver mval) {
    if (mval == LCP_CUSTOM)
        return;

    lcp_solver_type = mval;

    if (LCP_solver_speed)
        delete LCP_solver_speed;
    LCP_solver_speed = 0;
    if (LCP_solver_stab)
        delete LCP_solver_stab;
    LCP_solver_stab = 0;
    if (LCP_descriptor)
        delete LCP_descriptor;
    LCP_descriptor = 0;

    LCP_descriptor = new ChLcpSystemDescriptor;
    LCP_descriptor->SetNumThreads(parallel_thread_number);

    this->contact_container = ChSharedPtr<ChContactContainerDVI>(new ChContactContainerDVI());
    this->contact_container->SetSystem(this);

    switch (mval) {
        case LCP_ITERATIVE_SOR:
            LCP_solver_speed = new ChLcpIterativeSOR();
            LCP_solver_stab = new ChLcpIterativeSOR();
            break;
        case LCP_ITERATIVE_SYMMSOR:
            LCP_solver_speed = new ChLcpIterativeSymmSOR();
            LCP_solver_stab = new ChLcpIterativeSymmSOR();
            break;
        case LCP_SIMPLEX:
            LCP_solver_speed = new ChLcpSimplexSolver();
            LCP_solver_stab = new ChLcpSimplexSolver();
            break;
        case LCP_ITERATIVE_JACOBI:
            LCP_solver_speed = new ChLcpIterativeJacobi();
            LCP_solver_stab = new ChLcpIterativeJacobi();
            break;
        case LCP_ITERATIVE_SOR_MULTITHREAD:
            LCP_solver_speed = new ChLcpIterativeSORmultithread((char*)"speedLCP", parallel_thread_number);
            LCP_solver_stab = new ChLcpIterativeSORmultithread((char*)"posLCP", parallel_thread_number);
            break;
        case LCP_ITERATIVE_PMINRES:
            LCP_solver_speed = new ChLcpIterativePMINRES();
            LCP_solver_stab = new ChLcpIterativePMINRES();
            break;
        case LCP_ITERATIVE_BARZILAIBORWEIN:
            LCP_solver_speed = new ChLcpIterativeBB();
            LCP_solver_stab = new ChLcpIterativeBB();
            break;
        case LCP_ITERATIVE_PCG:
            LCP_solver_speed = new ChLcpIterativePCG();
            LCP_solver_stab = new ChLcpIterativePCG();
            break;
        case LCP_ITERATIVE_APGD:
            LCP_solver_speed = new ChIterativeAPGD();
            LCP_solver_stab = new ChIterativeAPGD();
            break;
        case LCP_ITERATIVE_MINRES:
            LCP_solver_speed = new ChLcpIterativeMINRES();
            LCP_solver_stab = new ChLcpIterativeMINRES();
            break;
        default:
            LCP_solver_speed = new ChLcpIterativeSymmSOR();
            LCP_solver_stab = new ChLcpIterativeSymmSOR();
            break;
    }
}

ChLcpSolver* ChSystem::GetLcpSolverSpeed() {
    // In case the solver is iterative, pre-configure it with the max. number of
    // iterations and with the convergence tolerance (convert the user-specified
    // tolerance for forces into a tolerance for impulses).
    if (ChLcpIterativeSolver* iter_solver = dynamic_cast<ChLcpIterativeSolver*>(LCP_solver_speed)) {
        iter_solver->SetMaxIterations(GetIterLCPmaxItersSpeed());
        iter_solver->SetTolerance(tol_force * step);
    }

    return LCP_solver_speed;
}

ChLcpSolver* ChSystem::GetLcpSolverStab() {
    // In case the solver is iterative, pre-configure it with the max. number of
    // iterations and with the convergence tolerance (convert the user-specified
    // tolerance for forces into a tolerance for impulses).
    if (ChLcpIterativeSolver* iter_solver = dynamic_cast<ChLcpIterativeSolver*>(LCP_solver_stab)) {
        iter_solver->SetMaxIterations(GetIterLCPmaxItersSpeed());
        iter_solver->SetTolerance(tol_force * step);
    }

    return LCP_solver_stab;
}

void ChSystem::SetIterLCPwarmStarting(bool usewarm) {
    if (ChLcpIterativeSolver* iter_solver_speed = dynamic_cast<ChLcpIterativeSolver*>(LCP_solver_speed)) {
        iter_solver_speed->SetWarmStart(usewarm);
    }
    if (ChLcpIterativeSolver* iter_solver_stab = dynamic_cast<ChLcpIterativeSolver*>(LCP_solver_stab)) {
        iter_solver_stab->SetWarmStart(usewarm);
    }
}

bool ChSystem::GetIterLCPwarmStarting() {
    if (ChLcpIterativeSolver* iter_solver_speed = dynamic_cast<ChLcpIterativeSolver*>(LCP_solver_speed)) {
        return iter_solver_speed->GetWarmStart();
    }
    return false;
}

void ChSystem::SetIterLCPomega(double momega) {
    if (ChLcpIterativeSolver* iter_solver_speed = dynamic_cast<ChLcpIterativeSolver*>(LCP_solver_speed)) {
        iter_solver_speed->SetOmega(momega);
    }
    if (ChLcpIterativeSolver* iter_solver_stab = dynamic_cast<ChLcpIterativeSolver*>(LCP_solver_stab)) {
        iter_solver_stab->SetOmega(momega);
    }
}

double ChSystem::GetIterLCPomega() {
    if (ChLcpIterativeSolver* iter_solver_speed = dynamic_cast<ChLcpIterativeSolver*>(LCP_solver_speed)) {
        return iter_solver_speed->GetOmega();
    }
    return 1.0;
}

void ChSystem::SetIterLCPsharpnessLambda(double momega) {
    if (ChLcpIterativeSolver* iter_solver_speed = dynamic_cast<ChLcpIterativeSolver*>(LCP_solver_speed)) {
        iter_solver_speed->SetSharpnessLambda(momega);
    }
    if (ChLcpIterativeSolver* iter_solver_stab = dynamic_cast<ChLcpIterativeSolver*>(LCP_solver_stab)) {
        iter_solver_stab->SetSharpnessLambda(momega);
    }
}

double ChSystem::GetIterLCPsharpnessLambda() {
    if (ChLcpIterativeSolver* iter_solver_speed = dynamic_cast<ChLcpIterativeSolver*>(LCP_solver_speed)) {
        return iter_solver_speed->GetSharpnessLambda();
    }
    return 1.0;
}

void ChSystem::SetParallelThreadNumber(int mthreads) {
    if (mthreads < 1)
        mthreads = 1;

    parallel_thread_number = mthreads;

    LCP_descriptor->SetNumThreads(mthreads);

    if (lcp_solver_type == LCP_ITERATIVE_SOR_MULTITHREAD) {
        ((ChLcpIterativeSORmultithread*)LCP_solver_speed)->ChangeNumberOfThreads(mthreads);
        ((ChLcpIterativeSORmultithread*)LCP_solver_stab)->ChangeNumberOfThreads(mthreads);
    }
}

// Plug-in components configuration

void ChSystem::ChangeLcpSystemDescriptor(ChLcpSystemDescriptor* newdescriptor) {
    assert(newdescriptor);
    if (this->LCP_descriptor)
        delete (this->LCP_descriptor);
    this->LCP_descriptor = newdescriptor;
}
void ChSystem::ChangeLcpSolverSpeed(ChLcpSolver* newsolver) {
    assert(newsolver);
    if (this->LCP_solver_speed)
        delete (this->LCP_solver_speed);
    this->LCP_solver_speed = newsolver;
    this->lcp_solver_type = LCP_CUSTOM;
}

void ChSystem::ChangeLcpSolverStab(ChLcpSolver* newsolver) {
    assert(newsolver);
    if (this->LCP_solver_stab)
        delete (this->LCP_solver_stab);
    this->LCP_solver_stab = newsolver;
    this->lcp_solver_type = LCP_CUSTOM;
}

void ChSystem::ChangeContactContainer(ChSharedPtr<ChContactContainerBase> newcontainer) {
    assert(newcontainer);

    this->contact_container = newcontainer;
    this->contact_container->SetSystem(this);
}

void ChSystem::ChangeCollisionSystem(ChCollisionSystem* newcollsystem) {
    assert(this->GetNbodies() == 0);
    assert(newcollsystem);
    if (this->collision_system)
        delete (this->collision_system);
    this->collision_system = newcollsystem;
}

// JS commands

int ChSystem::SetScriptForStartFile(const std::string& mfile) {
    if (!this->scriptEngine)
        return 0;
    this->scriptForStartFile = mfile;
    this->scriptForStart = this->scriptEngine->CreateScript();
    return this->scriptEngine->FileToScript(*this->scriptForStart, mfile.c_str());
}
int ChSystem::SetScriptForUpdateFile(const std::string& mfile) {
    if (!this->scriptEngine)
        return 0;
    this->scriptForUpdateFile = mfile;
    this->scriptForUpdate = this->scriptEngine->CreateScript();
    return this->scriptEngine->FileToScript(*this->scriptForUpdate, mfile.c_str());
}
int ChSystem::SetScriptForStepFile(const std::string& mfile) {
    if (!this->scriptEngine)
        return 0;
    this->scriptForStepFile = mfile;
    this->scriptForStep = this->scriptEngine->CreateScript();
    return this->scriptEngine->FileToScript(*this->scriptForStep, mfile.c_str());
}
int ChSystem::SetScriptFor3DStepFile(const std::string& mfile) {
    if (!this->scriptEngine)
        return 0;
    this->scriptFor3DStepFile = mfile;
    this->scriptFor3DStep = this->scriptEngine->CreateScript();
    return this->scriptEngine->FileToScript(*this->scriptFor3DStep, mfile.c_str());
}

int ChSystem::ExecuteScriptForStart() {
    if (!this->scriptEngine)
        return 0;
    return this->scriptEngine->ExecuteScript(*this->scriptForStart);
}
int ChSystem::ExecuteScriptForUpdate() {
    if (!this->scriptEngine)
        return 0;
    return this->scriptEngine->ExecuteScript(*this->scriptForUpdate);
}
int ChSystem::ExecuteScriptForStep() {
    if (!this->scriptEngine)
        return 0;
    return this->scriptEngine->ExecuteScript(*this->scriptForStep);
}
int ChSystem::ExecuteScriptFor3DStep() {
    if (!this->scriptEngine)
        return 0;
    return this->scriptEngine->ExecuteScript(*this->scriptFor3DStep);
}

// PROBE STUFF

int ChSystem::RecordAllProbes() {
    int pcount = 0;

    for (unsigned int ip = 0; ip < probelist.size(); ++ip)  // ITERATE on probes
    {
        ChSharedPtr<ChProbe> Ppointer = probelist[ip];

        Ppointer->Record(this->GetChTime());
    }

    return pcount;
}

int ChSystem::ResetAllProbes() {
    int pcount = 0;

    for (unsigned int ip = 0; ip < probelist.size(); ++ip)  // ITERATE on probes
    {
        ChSharedPtr<ChProbe> Ppointer = probelist[ip];

        Ppointer->Reset();
    }

    return pcount;
}

// CONTROLS STUFF

int ChSystem::ExecuteControlsForStart() {
    for (unsigned int ip = 0; ip < controlslist.size(); ++ip)  // ITERATE on controls
    {
        ChSharedPtr<ChControls> Cpointer = controlslist[ip];

        Cpointer->ExecuteForStart();
    }
    return TRUE;
}

int ChSystem::ExecuteControlsForUpdate() {
    for (unsigned int ip = 0; ip < controlslist.size(); ++ip)  // ITERATE on controls
    {
        ChSharedPtr<ChControls> Cpointer = controlslist[ip];

        Cpointer->ExecuteForUpdate();
    }
    return TRUE;
}

int ChSystem::ExecuteControlsForStep() {
    for (unsigned int ip = 0; ip < controlslist.size(); ++ip)  // ITERATE on controls
    {
        ChSharedPtr<ChControls> Cpointer = controlslist[ip];

        Cpointer->ExecuteForStep();
    }
    return TRUE;
}

int ChSystem::ExecuteControlsFor3DStep() {
    for (unsigned int ip = 0; ip < controlslist.size(); ++ip)  // ITERATE on controls
    {
        ChSharedPtr<ChControls> Cpointer = controlslist[ip];

        Cpointer->ExecuteFor3DStep();
    }
    return TRUE;
}

//
// HIERARCHY HANDLERS
//



void ChSystem::AddProbe(ChSharedPtr<ChProbe>& newprobe) {
    assert(std::find<std::vector<ChSharedPtr<ChProbe> >::iterator>(probelist.begin(), probelist.end(), newprobe) == probelist.end());

    // newprobe->SetSystem (this);
    probelist.push_back(newprobe);
}

void ChSystem::AddControls(ChSharedPtr<ChControls>& newcontrols) {
    assert(std::find<std::vector<ChSharedPtr<ChControls> >::iterator>(controlslist.begin(), controlslist.end(), newcontrols) == controlslist.end());

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
    std::vector<ChSharedPtr<ChLink> > toremove;

    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];

        if (ChSharedPtr<ChLinkMarkers> malink = Lpointer.DynamicCastTo<ChLinkMarkers>() ) {
            ChSharedPtr<ChMarker> shm1 = SearchMarker(malink->GetMarkID1());
            ChSharedPtr<ChMarker> shm2 = SearchMarker(malink->GetMarkID2());
            ChMarker* mm1 = shm1.get_ptr();
            ChMarker* mm2 = shm1.get_ptr();
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

//////
////// PREFERENCES

void ChSystem::SetIntegrationType(eCh_integrationType m_integration) {
    if (m_integration == integration_type)
        return;
    if (m_integration == INT_CUSTOM__)
        return;

    // set integration scheme:
    integration_type = m_integration;

    // plug in the new required timestepper
    // (the previous will be automatically deallocated thanks to shared pointers)
    switch (integration_type) {
        case INT_ANITESCU:
            this->timestepper =
                ChSharedPtr<ChTimestepper>();  // null because Integrate_Y_impulse will fallback to old code
            break;
        case INT_TASORA:
            this->timestepper =
                ChSharedPtr<ChTimestepper>();  // null because Integrate_Y_impulse will fallback to old code
            break;
        case INT_EULER_IMPLICIT:
            this->timestepper = ChSharedPtr<ChTimestepperEulerImplicit>(new ChTimestepperEulerImplicit(this));
            (this->timestepper.DynamicCastTo<ChTimestepperEulerImplicit>())->SetMaxiters(4);
            break;
        case INT_EULER_IMPLICIT_LINEARIZED:
            this->timestepper =
                ChSharedPtr<ChTimestepperEulerImplicitLinearized>(new ChTimestepperEulerImplicitLinearized(this));
            break;
        case INT_EULER_IMPLICIT_PROJECTED:
            this->timestepper =
                ChSharedPtr<ChTimestepperEulerImplicitProjected>(new ChTimestepperEulerImplicitProjected(this));
            break;
        case INT_TRAPEZOIDAL:
            this->timestepper = ChSharedPtr<ChTimestepperTrapezoidal>(new ChTimestepperTrapezoidal(this));
            (this->timestepper.DynamicCastTo<ChTimestepperTrapezoidal>())->SetMaxiters(4);
            break;
        case INT_TRAPEZOIDAL_LINEARIZED:
            this->timestepper =
                ChSharedPtr<ChTimestepperTrapezoidalLinearized>(new ChTimestepperTrapezoidalLinearized(this));
            (this->timestepper.DynamicCastTo<ChTimestepperTrapezoidalLinearized>())->SetMaxiters(4);
            break;
        case INT_HHT:
            this->timestepper = ChSharedPtr<ChTimestepperHHT>(new ChTimestepperHHT(this));
            (this->timestepper.DynamicCastTo<ChTimestepperHHT>())->SetMaxiters(4);
            break;
        case INT_HEUN:
            this->timestepper = ChSharedPtr<ChTimestepperHeun>(new ChTimestepperHeun(this));
            break;
        case INT_RUNGEKUTTA45:
            this->timestepper = ChSharedPtr<ChTimestepperRungeKuttaExpl>(new ChTimestepperRungeKuttaExpl(this));
            break;
        case INT_EULER_EXPLICIT:
            this->timestepper = ChSharedPtr<ChTimestepperEulerExplIIorder>(new ChTimestepperEulerExplIIorder(this));
            break;
        case INT_LEAPFROG:
            this->timestepper = ChSharedPtr<ChTimestepperLeapfrog>(new ChTimestepperLeapfrog(this));
            break;
        case INT_NEWMARK:
            this->timestepper = ChSharedPtr<ChTimestepperNewmark>(new ChTimestepperNewmark(this));
            break;
        default:
            throw ChException("SetIntegrationType: timestepper not supported");
    }
}



bool ChSystem::ManageSleepingBodies() {

    if (!this->GetUseSleeping())
        return 0;

    // STEP 1: 
    // See if some body could change from no sleep-> sleep

    for (int ip = 0; ip < bodylist.size(); ++ip)  {
        // mark as 'could sleep' candidate
        bodylist[ip]->TrySleeping();
    }

    // STEP 2:
    // See if some sleeping or potential sleeping body is touching a non sleeping one, 
    // if so, set to no sleep.

    // Make this class for iterating through contacts 

    class _wakeup_reporter_class : public ChReportContactCallback2 {
      public:
        /// Callback, used to report contact points already added to the container.
        /// This must be implemented by a child class of ChReportContactCallback.
        /// If returns false, the contact scanning will be stopped.
        virtual bool ReportContactCallback2(
            const ChVector<>& pA,             ///< get contact pA
            const ChVector<>& pB,             ///< get contact pB
            const ChMatrix33<>& plane_coord,  ///< get contact plane coordsystem (A column 'X' is contact normal)
            const double& distance,           ///< get contact distance
            const ChVector<>& react_forces,   ///< get react.forces (if already computed). In coordsystem 'plane_coord'
            const ChVector<>& react_torques,  ///< get react.torques, if rolling friction (if already computed).
            ChContactable* contactobjA,  ///< get model A (note: some containers may not support it and could be zero!)
            ChContactable* contactobjB   ///< get model B (note: some containers may not support it and could be zero!)
            ) {
            if (!(contactobjA && contactobjB))
                return true;
            ChBody* b1 = dynamic_cast<ChBody*>(contactobjA);
            ChBody* b2 = dynamic_cast<ChBody*>(contactobjB);
            if (!(b1 && b2))
                return true;
            bool sleep1 = b1->GetSleeping();
            bool sleep2 = b2->GetSleeping();
            bool could_sleep1 = b1->BFlagGet(BF_COULDSLEEP);
            bool could_sleep2 = b2->BFlagGet(BF_COULDSLEEP);
            bool ground1 = b1->GetBodyFixed();
            bool ground2 = b2->GetBodyFixed();
            if (sleep1 && !(sleep2||could_sleep2) && !ground2) {
                b1->SetSleeping(false);
                need_Setup_A = true;
            }
            if (sleep2 && !(sleep1||could_sleep1) && !ground1) {
                b2->SetSleeping(false);
                need_Setup_A = true;
            }
            if (could_sleep1 && !(sleep2||could_sleep2) && !ground2) {
                b1->BFlagSet(BF_COULDSLEEP,false);
            }
            if (could_sleep2 && !(sleep1||could_sleep1) && !ground1) {
                b2->BFlagSet(BF_COULDSLEEP,false);
            }
            this->someone_sleeps = sleep1 | sleep2 | this->someone_sleeps;

            return true;  // to continue scanning contacts
        }

        // Data
        bool someone_sleeps;
        bool need_Setup_A;
    };

    _wakeup_reporter_class my_waker;
    my_waker.need_Setup_A=false;

    bool need_Setup_L = false;

    for (int i = 0; i < 1; i++)  //***TO DO*** reconfigurable number of wakeup cycles
    {
        my_waker.someone_sleeps = false;

        // scan all links and wake connected bodies
        for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
        {
            ChSharedPtr<ChLink> Lpointer = linklist[ip];

            if (Lpointer->IsRequiringWaking()) {
                ChBody* b1 = dynamic_cast<ChBody*>(Lpointer->GetBody1());
                ChBody* b2 = dynamic_cast<ChBody*>(Lpointer->GetBody2());
                if (b1&&b2) {
                    bool sleep1 = b1->GetSleeping();
                    bool sleep2 = b2->GetSleeping();
                    bool could_sleep1 = b1->BFlagGet(BF_COULDSLEEP);
                    bool could_sleep2 = b2->BFlagGet(BF_COULDSLEEP);
                    if (sleep1 && !(sleep2||could_sleep2)) {
                        b1->SetSleeping(false);
                        need_Setup_L = true;
                    }
                    if (sleep2 && !(sleep1||could_sleep1)) {
                        b2->SetSleeping(false);
                        need_Setup_L = true;
                    }
                    if (could_sleep1 && !(sleep2||could_sleep2)) {
                        b1->BFlagSet(BF_COULDSLEEP,false);
                    }
                    if (could_sleep2 && !(sleep1||could_sleep1)) {
                        b2->BFlagSet(BF_COULDSLEEP,false);
                    }
                }
            }
        }

        // scan all contacts and wake neighbouring bodies
        this->contact_container->ReportAllContacts2(&my_waker);

        // bailout wakeup cycle prematurely, if all bodies are not sleeping
        if (!my_waker.someone_sleeps)
            break;
    }

    /// If some body still must change from no sleep-> sleep, do it
    int need_Setup_B = 0;
    for (int ip = 0; ip < bodylist.size(); ++ip)  {
        if (bodylist[ip]->BFlagGet(BF_COULDSLEEP)) {
            bodylist[ip]->SetSleeping(true);
            ++need_Setup_B;
        }
    }

    // if some body has been activated/deactivated because of sleep state changes, 
    // the offsets and DOF counts must be updated:
    if (my_waker.need_Setup_A || need_Setup_B || need_Setup_L) {
        this->Setup();
        return true;
    }
    return false;
}



///////////////////////////////
/////////
/////////  LCP BOOKKEEPING
/////////


void ChSystem::LCPprepare_inject(ChLcpSystemDescriptor& mdescriptor) {

    mdescriptor.BeginInsertion();  // This resets the vectors of constr. and var. pointers.

    this->InjectConstraints(mdescriptor);
    this->InjectVariables(mdescriptor);
    this->InjectKRMmatrices(mdescriptor);

    mdescriptor.EndInsertion();
}


void ChSystem::LCPprepare_reset() {
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        Lpointer->ConstraintsBiReset();
    }
    for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        Bpointer->VariablesFbReset();
    }
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];
        PHpointer->VariablesFbReset();
        PHpointer->ConstraintsBiReset();
    }
    this->contact_container->ConstraintsBiReset();
}

void ChSystem::LCPprepare_load(bool load_jacobians,
                               bool load_Mv,
                               double F_factor,
                               double K_factor,
                               double R_factor,
                               double M_factor,
                               double Ct_factor,
                               double C_factor,
                               double recovery_clamp,
                               bool do_clamp) {
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];

        if (C_factor)
            Lpointer->ConstraintsBiLoad_C(C_factor, recovery_clamp, do_clamp);
        if (Ct_factor)
            Lpointer->ConstraintsBiLoad_Ct(Ct_factor);  // Ct
        if (load_Mv) {
            Lpointer->VariablesQbLoadSpeed();    //   v_old
            Lpointer->VariablesFbIncrementMq();  // M*v_old
        }
        if (load_jacobians)
            Lpointer->ConstraintsLoadJacobians();
        if (F_factor) {
                Lpointer->ConstraintsFbLoadForces(F_factor);  // f*dt
            }
        }

    for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (F_factor)
            Bpointer->VariablesFbLoadForces(F_factor);  // f*dt
        if (load_Mv) {
            Bpointer->VariablesQbLoadSpeed();    //   v_old
            Bpointer->VariablesFbIncrementMq();  // M*v_old
        }
    }

    // Radu:
    //   The loop below is not immediately parallelizable because of items such as
    //   ChShaftsTorqueBase which add torques to other items' variables.  As such,
    //   there is potential contention with the various VariablesFbLoadForces and
    //   VariablesFbIncrementMq functions.
    //
    //   While it would be possible to protect the calls to the above two functions
    //   in additional critical sections, more experimentation is needed to decide
    //   if it's worth it.

    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];
        if (F_factor)
            PHpointer->VariablesFbLoadForces(F_factor);  // f*dt
        if (load_Mv) {
            PHpointer->VariablesQbLoadSpeed();    //   v_old
            PHpointer->VariablesFbIncrementMq();  // M*v_old
        }
        if (C_factor)
            PHpointer->ConstraintsBiLoad_C(C_factor, recovery_clamp, do_clamp);
        if (Ct_factor)
            PHpointer->ConstraintsBiLoad_Ct(Ct_factor);  // Ct
        if (load_jacobians)
            PHpointer->ConstraintsLoadJacobians();
        if (K_factor || R_factor || M_factor)
            PHpointer->KRMmatricesLoad(K_factor, R_factor, M_factor);
        if (F_factor) {
            PHpointer->ConstraintsFbLoadForces(F_factor);  // f*dt
        }
    }

    if (C_factor)
        contact_container->ConstraintsBiLoad_C(C_factor, recovery_clamp, do_clamp);
    if (F_factor)
        contact_container->ConstraintsFbLoadForces(F_factor);  // f*dt
    if (load_jacobians)
        contact_container->ConstraintsLoadJacobians();
}



void ChSystem::LCPprepare_Li_from_speed_cache() {
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        Lpointer->ConstraintsLiLoadSuggestedSpeedSolution();
    }
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];
        PHpointer->ConstraintsLiLoadSuggestedSpeedSolution();
    }
    this->contact_container->ConstraintsLiLoadSuggestedSpeedSolution();
}

void ChSystem::LCPprepare_Li_from_position_cache() {
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        Lpointer->ConstraintsLiLoadSuggestedPositionSolution();
    }
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];
        PHpointer->ConstraintsLiLoadSuggestedPositionSolution();
    }
    this->contact_container->ConstraintsLiLoadSuggestedPositionSolution();
}

void ChSystem::LCPresult_Li_into_speed_cache() {
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        Lpointer->ConstraintsLiFetchSuggestedSpeedSolution();
    }
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];
        PHpointer->ConstraintsLiFetchSuggestedSpeedSolution();
    }
    this->contact_container->ConstraintsLiFetchSuggestedSpeedSolution();
}

void ChSystem::LCPresult_Li_into_position_cache() {
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        Lpointer->ConstraintsLiFetchSuggestedPositionSolution();
    }
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];
        PHpointer->ConstraintsLiFetchSuggestedPositionSolution();
    }
    this->contact_container->ConstraintsLiFetchSuggestedPositionSolution();
}

void ChSystem::LCPresult_Li_into_reactions(double mfactor) {
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        Lpointer->ConstraintsFetch_react(mfactor);
    }
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];
        PHpointer->ConstraintsFetch_react(mfactor);
    }
    this->contact_container->ConstraintsFetch_react(mfactor);
}



//////////////////////////////////
////////
//////// CHPHYSICS ITEM INTERFACE
////////




//
// SETUP 
//
// Set all  offsets in position/speed global vectors, for all items.
// Count all bodies and links, etc, compute &set dof for statistics,
// allocates or reallocate bookkeeping data/vectors, if any,


void ChSystem::Setup() {
    events->Record(CHEVENT_SETUP);

    // inherit the parent class 
    // (compute offsets of bodies, links, etc.) 
    ChAssembly::Setup();

    // also compute offsets for contact container
    {
        contact_container->SetOffset_L(this->offset_L+ndoc_w);

        ndoc_w   += contact_container->GetDOC();
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
           ChState           test_x(this->GetNcoords_x(), this);
           ChStateDelta      test_v(this->GetNcoords_w(), this);
           ChStateDelta      test_a(this->GetNcoords_w(), this);
           ChVectorDynamic<> test_L(this->GetNconstr());
           double poison_x = -8888.888;
           double poison_v = -9999.999;
           double poison_a = -7777.777;
           double poison_L = 55555.555;
           double test_T;
           test_x.FillElem(poison_x); // poison x
           test_v.FillElem(poison_v); // poison v
           test_a.FillElem(poison_a); // poison a
           test_L.FillElem(poison_L); // poison L
           this->StateGather(test_x, test_v, test_T);
           this->StateGatherAcceleration(test_a);
           this->StateGatherReactions(test_L);
           for (int i= 0; i< test_x.GetRows(); ++i)
               assert(test_x(i)!=poison_x);  // if your debugger breaks here, some ChPhysicsItem has a wrong implementation of offsets or DOFs for positions 
           for (int i= 0; i< test_v.GetRows(); ++i)
               assert(test_v(i)!=poison_v);  // if your debugger breaks here, some ChPhysicsItem has a wrong implementation of offsets or DOFs for velocities
           for (int i= 0; i< test_a.GetRows(); ++i)
               assert(test_a(i)!=poison_a);  // if your debugger breaks here, some ChPhysicsItem has a wrong implementation of offsets or DOFs for accelerations
           for (int i= 0; i< test_L.GetRows(); ++i)
               assert(test_L(i)!=poison_L);  // if your debugger breaks here, some ChPhysicsItem has a wrong implementation of offsets or DOFs for reaction forces
       }   
    #endif // _DEBUG
}

//
// UPDATE
//
// - all physical items (bodies, links,etc.) are updated,
//   also updating their auxiliary variables (rot.matrices, etc.).
// - updates all forces  (automatic, as children of bodies)
// - updates all markers (automatic, as children of bodies).

void ChSystem::Update(bool update_assets) {

    timer_update.start();  // Timer for profiling

    events->Record(CHEVENT_UPDATE);  // Record an update event

    // Executes the "forUpdate" script, if any
    ExecuteScriptForUpdate();
    // Executes the "forUpdate" script
    // in all controls of controlslist
    ExecuteControlsForUpdate();

    // Inherit parent class
    // (recursively update sub objects bodies, links, etc)
    ChAssembly::Update(update_assets);

    // Update all contacts, if any
    this->contact_container->Update(ChTime, update_assets);  

    timer_update.stop();
}



void ChSystem::IntStateGather(const unsigned int off_x,  ///< offset in x state vector
                                ChState& x,                ///< state vector, position part
                                const unsigned int off_v,  ///< offset in v state vector
                                ChStateDelta& v,           ///< state vector, speed part
                                double& T)                 ///< time
{
    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateGather(off_x, x, off_v, v, T);
    // Use also on contact container:
    contact_container->IntStateGather(displ_x + contact_container->GetOffset_x(), x, displ_v + contact_container->GetOffset_w(), v, T);
}


void ChSystem::IntStateScatter(const unsigned int off_x,  ///< offset in x state vector
                                 const ChState& x,          ///< state vector, position part
                                 const unsigned int off_v,  ///< offset in v state vector
                                 const ChStateDelta& v,     ///< state vector, speed part
                                 const double T)            ///< time
{
    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateScatter(off_x,  x,  off_v, v, T);
    // Use also on contact container:
    contact_container->IntStateScatter(displ_x + contact_container->GetOffset_x(), x, displ_v + contact_container->GetOffset_w(), v, T);
}


void ChSystem::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) 
{
    unsigned int displ_a = off_a - this->offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateGatherAcceleration(off_a,  a);
    // Use also on contact container:
    contact_container->IntStateGatherAcceleration(displ_a + contact_container->GetOffset_w(), a);
}

/// From state derivative (acceleration) to system, sometimes might be needed
void ChSystem::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) 
{
    unsigned int displ_a = off_a - this->offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateScatterAcceleration(off_a,  a);
    // Use also on contact container:
    contact_container->IntStateScatterAcceleration(displ_a + contact_container->GetOffset_w(), a);
}

/// From system to reaction forces (last computed) - some timestepper might need this
void ChSystem::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) 
{
    unsigned int displ_L = off_L - this->offset_L;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateGatherReactions(off_L,  L);
    // Use also on contact container:
    contact_container->IntStateGatherReactions(displ_L + contact_container->GetOffset_L(), L);
}

/// From reaction forces to system, ex. store last computed reactions in ChLink objects for plotting etc.
void ChSystem::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) 
{
    unsigned int displ_L = off_L - this->offset_L;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateScatterReactions(off_L,  L);
    // Use also on contact container:
    contact_container->IntStateScatterReactions(displ_L + contact_container->GetOffset_L(), L);
}

void ChSystem::IntStateIncrement(const unsigned int off_x,  ///< offset in x state vector
                                   ChState& x_new,            ///< state vector, position part, incremented result
                                   const ChState& x,          ///< state vector, initial position part
                                   const unsigned int off_v,  ///< offset in v state vector
                                   const ChStateDelta& Dv)    ///< state vector, increment
{
    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntStateIncrement(off_x, x_new, x, off_v, Dv);
    // Use also on contact container:
    contact_container->IntStateIncrement(displ_x + contact_container->GetOffset_x(), x_new, x, displ_v + contact_container->GetOffset_w(), Dv);
}

void ChSystem::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                   ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                   const double c           ///< a scaling factor
                                   ) {
    unsigned int displ_v = off  - this->offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntLoadResidual_F(off, R, c);
    // Use also on contact container:
    contact_container->IntLoadResidual_F(displ_v + contact_container->GetOffset_w(), R, c);
}

void ChSystem::IntLoadResidual_Mv(const unsigned int off,   ///< offset in R residual
                                    ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                    const ChVectorDynamic<>& w,  ///< the w vector
                                    const double c               ///< a scaling factor
                                    ) {
    unsigned int displ_v = off  - this->offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntLoadResidual_Mv(off, R, w, c);
    // Use also on contact container:
    contact_container->IntLoadResidual_Mv(displ_v + contact_container->GetOffset_w(), R, w, c);
}

void ChSystem::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                     ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                     const ChVectorDynamic<>& L,  ///< the L vector
                                     const double c               ///< a scaling factor
                                     ) {
    unsigned int displ_L = off_L  - this->offset_L;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntLoadResidual_CqL(off_L, R, L, c);
    // Use also on contact container:
    contact_container->IntLoadResidual_CqL(displ_L + contact_container->GetOffset_L(), R, L, c);
}

void ChSystem::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                     ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                     const double c,            ///< a scaling factor
                                     bool do_clamp,             ///< apply clamping to c*C?
                                     double recovery_clamp      ///< value for min/max clamping of c*C
                                     ) {
    unsigned int displ_L = off_L  - this->offset_L;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntLoadConstraint_C(off_L, Qc, c, do_clamp, recovery_clamp);
    // Use also on contact container:
    contact_container->IntLoadConstraint_C(displ_L + contact_container->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
}

void ChSystem::IntLoadConstraint_Ct(const unsigned int off_L,  ///< offset in Qc residual
                                      ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*Ct
                                      const double c             ///< a scaling factor
                                      ) {
    unsigned int displ_L = off_L  - this->offset_L;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntLoadConstraint_Ct(off_L, Qc, c);
    // Use also on contact container:
    contact_container->IntLoadConstraint_Ct(displ_L + contact_container->GetOffset_L(), Qc, c);
}

void ChSystem::IntToLCP(const unsigned int off_v,  ///< offset in v, R
                          const ChStateDelta& v,
                          const ChVectorDynamic<>& R,
                          const unsigned int off_L,  ///< offset in L, Qc
                          const ChVectorDynamic<>& L,
                          const ChVectorDynamic<>& Qc) {
    unsigned int displ_L = off_L  - this->offset_L;
    unsigned int displ_v = off_v  - this->offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntToLCP(off_v,  v, R, off_L,  L, Qc);
    // Use also on contact container:
    contact_container->IntToLCP(displ_v + contact_container->GetOffset_w(),v,R, displ_L + contact_container->GetOffset_L(),L,Qc);
}

void ChSystem::IntFromLCP(const unsigned int off_v,  ///< offset in v
                            ChStateDelta& v,
                            const unsigned int off_L,  ///< offset in L
                            ChVectorDynamic<>& L) {
    unsigned int displ_L = off_L  - this->offset_L;
    unsigned int displ_v = off_v  - this->offset_w;

    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::IntFromLCP(off_v,  v, off_L, L);
    // Use also on contact container:
    contact_container->IntFromLCP(displ_v + contact_container->GetOffset_w(),v, displ_L + contact_container->GetOffset_L(),L);
}

////
void ChSystem::InjectVariables(ChLcpSystemDescriptor& mdescriptor) {
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

void ChSystem::InjectConstraints(ChLcpSystemDescriptor& mdescriptor) {
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

void ChSystem::ConstraintsLiLoadSuggestedSpeedSolution() {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::ConstraintsLiLoadSuggestedSpeedSolution();
    // Use also on contact container:
    contact_container->ConstraintsLiLoadSuggestedSpeedSolution();
}

void ChSystem::ConstraintsLiLoadSuggestedPositionSolution() {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::ConstraintsLiLoadSuggestedPositionSolution();
    // Use also on contact container:
    contact_container->ConstraintsLiLoadSuggestedPositionSolution();
}

void ChSystem::ConstraintsLiFetchSuggestedSpeedSolution() {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::ConstraintsLiFetchSuggestedSpeedSolution();
    // Use also on contact container:
    contact_container->ConstraintsLiFetchSuggestedSpeedSolution();
}

void ChSystem::ConstraintsLiFetchSuggestedPositionSolution() {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::ConstraintsLiFetchSuggestedPositionSolution();
    // Use also on contact container:
    contact_container->ConstraintsLiFetchSuggestedPositionSolution();
}

void ChSystem::ConstraintsFetch_react(double factor) {
    // Inherit: operate parent method on sub objects (bodies, links, etc.)
    ChAssembly::ConstraintsFetch_react(factor);
    // Use also on contact container:
    contact_container->ConstraintsFetch_react(factor);
}

void ChSystem::InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor) {
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


//////////////////////////////////
////////
////////    TIMESTEPPER INTERFACE
////////

/// From system to state y={x,v}
void ChSystem::StateGather(ChState& x, ChStateDelta& v, double& T) {

    this->IntStateGather(0, x,0, v, T);
}

/// From state Y={x,v} to system.
void ChSystem::StateScatter(const ChState& x, const ChStateDelta& v, const double T) {

    this->IntStateScatter(0,x, 0,v, T);

    this->Update();  //***TODO*** optimize because maybe IntStateScatter above might have already called Update?
}

/// From system to state derivative (acceleration), some timesteppers might need last computed accel.
void ChSystem::StateGatherAcceleration(ChStateDelta& a) {

    this->IntStateGatherAcceleration(0, a);
}

/// From state derivative (acceleration) to system, sometimes might be needed
void ChSystem::StateScatterAcceleration(const ChStateDelta& a) {
    
    this->IntStateScatterAcceleration(0, a);
}

/// From system to reaction forces (last computed) - some timestepper might need this
void ChSystem::StateGatherReactions(ChVectorDynamic<>& L) {
    
    this->IntStateGatherReactions(0, L);
}

/// From reaction forces to system, ex. store last computed reactions in ChLink objects for plotting etc.
void ChSystem::StateScatterReactions(const ChVectorDynamic<>& L) {
    
    this->IntStateScatterReactions(0, L);
}

/// Perform x_new = x + dx    for x in    Y = {x, dx/dt}
/// It takes care of the fact that x has quaternions, dx has angular vel etc.
/// NOTE: the system is not updated automatically after the state increment, so one might
/// need to call StateScatter() if needed.
void ChSystem::StateIncrementX(ChState& x_new,         ///< resulting x_new = x + Dx
                               const ChState& x,       ///< initial state x
                               const ChStateDelta& Dx  ///< state increment Dx
                               ) {
    this->IntStateIncrement(0, x_new, x, 0, Dx);
}

/// Assuming an explicit DAE in the form
///        M*a = F(x,v,t) + Cq'*L
///       C(x,t) = 0
/// this must compute the solution of the change Du (in a or v or x) to satisfy
/// the equation required in a Newton Raphson iteration for an
/// implicit integrator equation:
///  |Du| = [ G   Cq' ]^-1 * | R |
///  |DL|   [ Cq  0   ]      | Qc|
/// for residual R and  G = [ c_a*M + c_v*dF/dv + c_x*dF/dx ]
void ChSystem::StateSolveCorrection(ChStateDelta& Dv,             ///< result: computed Dv
                                    ChVectorDynamic<>& L,         ///< result: computed lagrangian multipliers, if any
                                    const ChVectorDynamic<>& R,   ///< the R residual
                                    const ChVectorDynamic<>& Qc,  ///< the Qc residual
                                    const double c_a,             ///< the factor in c_a*M
                                    const double c_v,             ///< the factor in c_v*dF/dv
                                    const double c_x,             ///< the factor in c_x*dF/dv
                                    const ChState& x,             ///< current state, x part
                                    const ChStateDelta& v,        ///< current state, v part
                                    const double T,               ///< current time T
                                    bool force_state_scatter  ///< if false, x,v and T are not scattered to the system,
                                    /// assuming that someone has done StateScatter just before
                                    ) {
    this->solvecount++;

    if (force_state_scatter)
        this->StateScatter(x, v, T);

    // R and Qc vectors  --> LCP sparse solver structures  (also sets L and Dv to warmstart)

    this->IntToLCP(0, Dv, R, 0, L, Qc);

    // G and Cq  matrices:  fill the LCP sparse solver structures:

    this->ConstraintsLoadJacobians();
    
    // M, K, R matrices:  fill the LCP sparse solver structures:

    if (c_a || c_v || c_x)
        this->KRMmatricesLoad(-c_x, -c_v, c_a); // for KRM blocks in ChLcpKblock objects: fill them
    this->LCP_descriptor->SetMassFactor(c_a); // for ChLcpVariable objects, that does not have ChLcpKblock: just use a coeff., to avoid duplicated data 


    // diagnostics:

    if (this->dump_matrices) {
        // GetLog() << "StateSolveCorrection R=" << R << "\n\n";
        // GetLog() << "StateSolveCorrection Qc="<< Qc << "\n\n";
        // GetLog() << "StateSolveCorrection X=" << x << "\n\n";
        // GetLog() << "StateSolveCorrection V=" << v << "\n\n";
        const char* numformat = "%.12g";
        char cprefix[100];
        sprintf(cprefix, "solve_%04d_%02d_", this->stepcount, this->solvecount);
        std::string sprefix(cprefix);

        this->LCP_descriptor->DumpLastMatrices(true,  sprefix.c_str());
        this->LCP_descriptor->DumpLastMatrices(false, sprefix.c_str());

        chrono::ChStreamOutAsciiFile file_x( (sprefix+"x_pre.dat").c_str() );
        file_x.SetNumFormat(numformat);
        ((ChMatrix<>)x).StreamOUTdenseMatlabFormat(file_x);

        chrono::ChStreamOutAsciiFile file_v( (sprefix+"v_pre.dat").c_str() );
        file_v.SetNumFormat(numformat);
        ((ChMatrix<>)v).StreamOUTdenseMatlabFormat(file_v);

        chrono::ChStreamOutAsciiFile file_R( (sprefix+"R.dat").c_str() );
        file_R.SetNumFormat(numformat);
        ((ChMatrix<>)R).StreamOUTdenseMatlabFormat(file_R); // already saved as f from DumpLastMatrices?

        chrono::ChStreamOutAsciiFile file_Qc( (sprefix+"Qc.dat").c_str() );
        file_Qc.SetNumFormat(numformat);
        ((ChMatrix<>)Qc).StreamOUTdenseMatlabFormat(file_Qc); // already saved as b from DumpLastMatrices?
    }

    // Solve the LCP problem!!!!!!!!

    timer_lcp.start();

    GetLcpSolverSpeed()->Solve(*this->LCP_descriptor);

    timer_lcp.stop();


    // Dv and L vectors  <-- LCP sparse solver structures

    this->IntFromLCP(0, Dv, 0, L);
    
    // diagnostics:

    if (this->dump_matrices) {
        const char* numformat = "%.12g";
        char cprefix[100];
        sprintf(cprefix, "solve_%04d_%02d_", this->stepcount, this->solvecount);
        std::string sprefix(cprefix);

        chrono::ChStreamOutAsciiFile file_Dv( (sprefix+"Dv.dat").c_str() );
        file_Dv.SetNumFormat(numformat);
        ((ChMatrix<>)Dv).StreamOUTdenseMatlabFormat(file_Dv);

        chrono::ChStreamOutAsciiFile file_L( (sprefix+"L.dat").c_str() );
        file_L.SetNumFormat(numformat);
        ((ChMatrix<>)L).StreamOUTdenseMatlabFormat(file_L);

        // GetLog() << "StateSolveCorrection Dv=" << Dv << "\n\n";
        // GetLog() << "StateSolveCorrection L="  << L << "\n\n";
    }
}

/// Increment a vector R with the term c*F:
///    R += c*F
void ChSystem::LoadResidual_F(ChVectorDynamic<>& R,  ///< result: the R residual, R += c*F
                              const double c         ///< a scaling factor
                              ) {
    this->IntLoadResidual_F(0, R, c);
}

/// Increment a vector R with a term that has M multiplied a given vector w:
///    R += c*M*w
void ChSystem::LoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                               const ChVectorDynamic<>& w,  ///< the w vector
                               const double c               ///< a scaling factor
                               ) {
    this->IntLoadResidual_Mv(0, R, w, c);
}

/// Increment a vectorR with the term Cq'*L:
///    R += c*Cq'*L
void ChSystem::LoadResidual_CqL(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                const ChVectorDynamic<>& L,  ///< the L vector
                                const double c               ///< a scaling factor
                                ) {
    this->IntLoadResidual_CqL(0, R, L, c);
}

/// Increment a vector Qc with the term C:
///    Qc += c*C
void ChSystem::LoadConstraint_C(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*C
                                const double c,         ///< a scaling factor
                                const bool mdo_clamp,   ///< enable optional clamping of Qc
                                const double mclam      ///< clamping value
                                ) {
    this->IntLoadConstraint_C(0, Qc, c, mdo_clamp, mclam);
}

/// Increment a vector Qc with the term Ct = partial derivative dC/dt:
///    Qc += c*Ct
void ChSystem::LoadConstraint_Ct(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*Ct
                                 const double c          ///< a scaling factor
                                 ) {
    this->IntLoadConstraint_Ct(0, Qc, c);
}


//////////////////////////////////
////////
////////    COLLISION OPERATIONS
////////

int ChSystem::GetNcontacts() {
    return this->contact_container->GetNcontacts();
}

void ChSystem::SynchronizeLastCollPositions() {
    for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];

        if (Bpointer->GetCollide())
            Bpointer->SynchronizeLastCollPos();
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
        this->contact_container->SetAddContactCallback(&mpointcallback);
    } else
        this->contact_container->SetAddContactCallback(0);

    // !!! Perform the collision detection ( broadphase and narrowphase ) !!!

    collision_system->Run();

    // Report and store contacts and/or proximities, if there are some
    // containers in the physic system. The default contact container
    // for ChBody and ChParticles is used always.

    collision_system->ReportContacts(this->contact_container.get_ptr());

    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];
        if (ChSharedPtr<ChContactContainerBase> mcontactcontainer = PHpointer.DynamicCastTo<ChContactContainerBase>()) {
            collision_system->ReportContacts(mcontactcontainer.get_ptr());
        }
        if (ChSharedPtr<ChProximityContainerBase> mproximitycontainer = PHpointer.DynamicCastTo<ChProximityContainerBase>()) {
            collision_system->ReportProximities(mproximitycontainer.get_ptr());
        }
    }

    // If some other collision engine could add further ChLinkContact into the list..
    for (size_t ic = 0; ic < collision_callbacks.size(); ic++)
        collision_callbacks[ic]->PerformCustomCollision(this);

    // Count the contacts of body-body type.
    this->ncontacts = this->contact_container->GetNcontacts();

    timer_collision_broad.stop();

    return mretC;
}

///////////////////////////////////
////////
////////   PHYSICAL OPERATIONS
////////

//  PERFORM AN INTEGRATION STEP.  ----
//
//  Advances a single time step.
//
//  Note that time step can be modified if some variable-time stepper is used.
//
//

int ChSystem::DoStepDynamics(double m_step) {
    this->step = m_step;
    return Integrate_Y();
}


int ChSystem::Integrate_Y() {
    ResetTimers();
    switch (integration_type) {
        case INT_ANITESCU:
            return Integrate_Y_impulse_Anitescu();
        case INT_TASORA:
            return Integrate_Y_impulse_Tasora();
        default:
            return Integrate_Y_timestepper();
    }

    return TRUE;
}

//
//  PERFORM ANITESCU INTEGRATION STEP  -IMPULSIVE METHOD-
//
//  ...but using the differential inclusion approach, better for
//  systems with contacts (use the Anitescu method, with stabilization)
//

int ChSystem::Integrate_Y_impulse_Anitescu() {
    GetLog() << "WARNING! The INT_ANITESCU timestepper is deprecated. Use the INT_EULER_IMPLICIT_LINEARIZED instead.\n";

    int ret_code = TRUE;

    timer_step.start();

    events->Record(CHEVENT_TIMESTEP);

    // Executes the "forStep" script, if any
    ExecuteScriptForStep();
    // Executes the "forStep" script
    // in all controls of controlslist
    ExecuteControlsForStep();

    this->stepcount++;
    this->solvecount = 0;

    // Compute contacts and create contact constraints
    ComputeCollisions();

    // Counts dofs, statistics, etc.
    Setup();

    // Update everything.
    // Note that we do not update visualization assets at this point.
    Update(false);

    // Re-wake the bodies that cannot sleep because they are in contact with
    // some body that is not in sleep state.
    ManageSleepingBodies();

    timer_lcp.start();

    //
    // Enforce velocity/impulses constraints ....................
    //

    // reset known-term vectors
    LCPprepare_reset();

    // fill LCP known-term vectors with proper terms (forces, etc.):
    //
    // | M+dt^2*K+dt*R -Cq'|*|v_new|- | [M]*v_old + f*dt      | = |0| ,  c>=0, l>=0, l*c=0;
    // | Cq              0 | |l    |  | -Ct +min(-C/dt,vlim)  |   |c|
    //

    LCPprepare_load(true,         // Cq,
                    true,         // adds [M]*v_old to the known vector
                    step,         // f*dt
                    step * step,  // dt^2*K  (nb only non-Schur based solvers support K matrix blocks)
                    step,         // dt*R   (nb only non-Schur based solvers support R matrix blocks)
                    1.0,          // M (for FEM with non-lumped masses, add their mass-matrixes)
                    1.0,          // Ct   (needed, for rheonomic motors)
                    1.0 / step,   // C/dt
                    max_penetration_recovery_speed,  // vlim, max penetrations recovery speed (positive for exiting)
                    true);                           // do above max. clamping on -C/dt

    // if warm start is used, can exploit cached multipliers from last step...
    LCPprepare_Li_from_speed_cache();

    // make vectors of variables and constraints, used by the following LCP solver
    LCPprepare_inject(*this->LCP_descriptor);

    // Solve the LCP problem.
    // Solution variables are new speeds 'v_new'
    GetLcpSolverSpeed()->Solve(*this->LCP_descriptor);

    timer_lcp.stop();

    // stores computed multipliers in constraint caches, maybe useful for warm starting next step
    LCPresult_Li_into_speed_cache();

    // updates the reactions of the constraint
    LCPresult_Li_into_reactions(1.0 / step);  // R = l/dt  , approximately

// perform an Eulero integration step (1st order stepping as pos+=v_new*dt)

    for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];

        // EULERO INTEGRATION: pos+=v_new*dt  (do not do this, if GPU already computed it)
        Bpointer->VariablesQbIncrementPosition(step);
        // Set body speed, and approximates the acceleration by differentiation.
        Bpointer->VariablesQbSetSpeed(step);

        // Now also updates all markers & forces
        Bpointer->Update(this->ChTime);
    }

    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];

        // EULERO INTEGRATION: pos+=v_new*dt  (do not do this, if GPU already computed it)
        PHpointer->VariablesQbIncrementPosition(step);
        // Set body speed, and approximates the acceleration by differentiation.
        PHpointer->VariablesQbSetSpeed(step);

        // Now also updates all markers & forces
        PHpointer->Update(this->ChTime);
    }

    this->ChTime = ChTime + step;

    // Executes custom processing at the end of step
    CustomEndOfStep();

    // If there are some probe objects in the probe list,
    // tell them to record their variables (ususally x-y couples)
    RecordAllProbes();

    // Time elapsed for step..
    timer_step.stop();

    return (ret_code);
}

//
//  PERFORM TASORA INTEGRATION STEP  -IMPULSIVE METHOD-
//
//  ...but using the differential inclusion approach, better for
//  systems with contacts (use the Tasora method, with separate
//  positional stabilization)
//

int ChSystem::Integrate_Y_impulse_Tasora() {
    GetLog() << "WARNING! The INT_TASORA timestepper is deprecated. Use the INT_EULER_IMPLICIT_PROJECTED instead.\n";

    int ret_code = TRUE;

    timer_step.start();

    events->Record(CHEVENT_TIMESTEP);

    // Executes the "forStep" script, if any
    ExecuteScriptForStep();
    // Executes the "forStep" script
    // in all controls of controlslist
    ExecuteControlsForStep();

    this->stepcount++;
    this->solvecount =0;

    // Compute contacts and create contact constraints
    ComputeCollisions();

    // Counts dofs, statistics, etc.
    Setup();

    // Update everything.
    // Note that we do not update visualization assets at this point.
    Update(false);

    // Re-wake the bodies that cannot sleep because they are in contact with
    // some body that is not in sleep state.
    ManageSleepingBodies();

    timer_lcp.reset();
    timer_lcp.start();

    // 1-
    // Enforce velocity/impulses constraints ....................
    //

    // reset known-term vectors
    LCPprepare_reset();

    // fill LCP known-term vectors with proper terms (forces, etc.):
    //
    // | M -Cq'|*|v_new|- | [M]*v_old + f*dt    | = |0| ,  c>=0, l>=0, l*c=0;
    // | Cq  0 | |l    |  |  - Ct +min(-C/dt,0) |   |c|
    //

    LCPprepare_load(true,         // Cq
                    true,         // adds [M]*v_old to the known vector
                    step,         // f*dt
                    step * step,  // dt^2*K  (nb only non-Schur based solvers support K matrix blocks)
                    step,         // dt*R   (nb only non-Schur based solvers support K matrix blocks)
                    1.0,          // M (for FEM with non-lumped masses, add their mass-matrices)
                    1.0,          // Ct      (needed, for rheonomic motors)
                    1.0 / step,   // C/dt
                    0.0,          // max constr.recovery speed (positive for exiting)
                    true);        // do above max. clamping on -C/dt

    // if warm start is used, can exploit cached multipliers from last step...
    LCPprepare_Li_from_speed_cache();

    // make vectors of variables and constraints, used by the following LCP solver
    LCPprepare_inject(*this->LCP_descriptor);

    // Solve the LCP problem.
    // Solution variables are new speeds 'v_new'

    GetLcpSolverSpeed()->Solve(*this->LCP_descriptor);

    // stores computed multipliers in constraint caches, maybe useful for warm starting next step
    LCPresult_Li_into_speed_cache();

    // updates the reactions of the constraint
    LCPresult_Li_into_reactions(1.0 / step);  // R = l/dt  , approximately

    // perform an Eulero integration step (1st order stepping as pos+=v_new*dt)

    for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];

        // EULERO INTEGRATION: pos+=v_new*dt
        Bpointer->VariablesQbIncrementPosition(step);
        // Set body speed, and approximates the acceleration by differentiation.
        Bpointer->VariablesQbSetSpeed(step);

        // Now also updates all markers & forces
        // Bpointer->UpdateALL(this->ChTime); // not needed - will be done later anyway
    }

    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];

        // EULERO INTEGRATION: pos+=v_new*dt
        PHpointer->VariablesQbIncrementPosition(step);
        // Set body speed, and approximates the acceleration by differentiation.
        PHpointer->VariablesQbSetSpeed(step);

        // Now also updates all markers & forces
        // PHpointer->UpdateALL(this->ChTime); // not needed - will be done later anyway
    }

    this->ChTime = ChTime + step;

    // 2-
    // Stabilize constraint positions ....................
    //

    // reset known-term vectors
    LCPprepare_reset();

    // Fill known-term vectors with proper terms 0 and -C :
    //
    // | M -Cq'|*|Dpos|- |0 |= |0| ,  c>=0, l>=0, l*c=0;
    // | Cq  0 | |l   |  |-C|  |c|
    //

    LCPprepare_load(false,   // Cq are already there..
                    false,   // no addition of M*v in known term
                    0,       // no forces
                    0,       // no K matrix
                    0,       // no R matrix
                    1.0,     // M (for FEM with non-lumped masses, add their mass-matrices)
                    0,       // no Ct term
                    1.0,     // C
                    0.0,     // recovery max speed (not used)
                    false);  // no clamping on -C term

    // if warm start is used, can exploit cached multipliers from last step...
    LCPprepare_Li_from_position_cache();

    // Solve the LCP problem.
    // Solution variables are 'Dpos', delta positions.

    GetLcpSolverStab()->Solve(*this->LCP_descriptor);

    // stores computed multipliers in constraint caches, maybe useful for warm starting next step
    LCPresult_Li_into_position_cache();

    {
        for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
        {
            ChSharedPtr<ChBody> Bpointer = bodylist[ip];

            Bpointer->VariablesQbIncrementPosition(1.0);  // pos+=Dpos
            // Now also updates all markers & forces
            Bpointer->Update(this->ChTime);
        }

        for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
        {
            ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];

            PHpointer->VariablesQbIncrementPosition(1.0);  // pos+=Dpos
            // Now also updates all markers & forces
            PHpointer->Update(this->ChTime);
        }
    }

    timer_lcp.stop();

    // Executes custom processing at the end of step
    CustomEndOfStep();

    // If there are some probe objects in the probe list,
    // tell them to record their variables (ususally x-y couples)
    RecordAllProbes();

    // Time elapsed for step..
    timer_step.stop();

    return (ret_code);
}

//
//  PERFORM INTEGRATION STEP  using pluggable timestepper
//

int ChSystem::Integrate_Y_timestepper() {
    int ret_code = TRUE;

    timer_step.start();

    events->Record(CHEVENT_TIMESTEP);

    // Executes the "forStep" script, if any
    ExecuteScriptForStep();
    // Executes the "forStep" script
    // in all controls of controlslist
    ExecuteControlsForStep();

    this->stepcount++;
    this->solvecount = 0;

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
    LCPprepare_inject(*this->LCP_descriptor);
    LCP_descriptor->UpdateCountsAndOffsets();

    timer_lcp.reset();

    // Set some settings in timestepper object
    this->timestepper->SetQcDoClamp(true);
    this->timestepper->SetQcClamping(this->max_penetration_recovery_speed);
    if (this->timestepper.IsType<ChTimestepperHHT>() || this->timestepper.IsType<ChTimestepperNewmark>())
        this->timestepper->SetQcDoClamp(false);

    // PERFORM TIME STEP HERE!
    this->timestepper->Advance(step);

    // Executes custom processing at the end of step
    CustomEndOfStep();

    // If there are some probe objects in the probe list,
    // tell them to record their variables (ususally x-y couples)
    RecordAllProbes();

    // Time elapsed for step..
    timer_step.stop();


    return (ret_code);
}

// **** SATISFY ALL COSTRAINT EQUATIONS WITH NEWTON
// **** ITERATION, UNTIL TOLERANCE SATISFIED, THEN UPDATE
// **** THE "Y" STATE WITH SetY (WHICH AUTOMATICALLY UPDATES
// **** ALSO AUXILIARY MATRICES).

int ChSystem::DoAssembly(int action, int mflags) {

    this->solvecount = 0;

    // Counts dofs, statistics, etc.
    Setup();

    // Update the system and all its components.
    // No need to update visualization assets here.
    Update(false);

    //
    // (1)--------  POSITION
    //
    if (action & ASS_POSITION) {
        for (int m_iter = 0; m_iter < maxiter; m_iter++) {
            if (mflags & ASF_COLLISIONS) {
                // Compute new contacts and create contact constraints
                ComputeCollisions();

                Setup();        // Counts dofs, statistics, etc.
                Update(false);  // Update everything (do not update visualization assets)
            }

            // Reset known-term vectors
            LCPprepare_reset();

            // Fill known-term vectors with proper terms 0 and -C :
            //
            // | M -Cq'|*|Dpos|- |0 |= |0| ,  c>=0, l>=0, l*c=0;
            // | Cq  0 | |l   |  |-C|  |c|
            //
            LCPprepare_load(true,    // calculate Jacobian Cq
                            false,   // no addition of M*v in known term
                            0,       // no forces
                            0,       // no K matrix
                            0,       // no R matrix
                            1.0,     // M (for FEM with non-lumped masses, add their mass-matrices)
                            0,       // no Ct term
                            1.0,     // C
                            0.0,     //
                            false);  // no clamping on -C/dt

            // Make the vectors of pointers to constraint and variables, for LCP solver
            LCPprepare_inject(*this->LCP_descriptor);

            // Check violation and exit Newton loop if reached tolerance.
            double max_res, max_LCPerr;
            this->LCP_descriptor->ComputeFeasabilityViolation(max_res, max_LCPerr);
            if (max_res <= this->tol)
                break;

            // Solve the LCP problem.
            // Solution variables are 'Dpos', delta positions.
            // Note: use settings of the 'speed' lcp solver (i.e. use max number
            // of iterations as you would use for the speed probl., if iterative solver)
            GetLcpSolverSpeed()->Solve(*this->LCP_descriptor);

            // Update bodies and other physics items at new positions

            for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
            {
                ChSharedPtr<ChBody> Bpointer = bodylist[ip];
                Bpointer->VariablesQbIncrementPosition(1.0);  // pos += Dpos
                Bpointer->Update(this->ChTime);
            }

            for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
            {
                ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];
                PHpointer->VariablesQbIncrementPosition(1.0);  // pos += Dpos
                PHpointer->Update(this->ChTime);
            }

            Update();  // Update everything

        }  // end loop Newton iterations
    }

    //
    // 2) -------- SPEEDS and ACCELERATIONS
    //
    if ((action & ASS_SPEED) || (action & ASS_ACCEL)) {
        // Save current value of the time step and max number of 'speed' iterations.
        double step_saved = step;
        int niter_saved = GetIterLCPmaxItersSpeed();

        // Use a small time step for assembly. Also, temporarily increase the max
        // number of iterations for the speed solver (to accommodate for the bad
        // initial guess)
        step = 1e-7;
        SetIterLCPmaxItersSpeed(5 * niter_saved);

        // Reset known-term vectors
        LCPprepare_reset();

        // Fill LCP known-term vectors with proper terms
        //
        // | M -Cq'|*|v_new|- | [M]*v_old +f*dt | = |0| ,  c>=0, l>=0, l*c=0;
        // | Cq  0 | |l    |  |  - Ct           |   |c|
        //
        LCPprepare_load(false,        // Cq are already there..
                        true,         // adds [M]*v_old to the known vector
                        step,         // f*dt
                        step * step,  // dt^2*K  (nb only non-Schur based solvers support K matrix blocks)
                        step,         // dt*R   (nb only non-Schur based solvers support K matrix blocks)
                        1.0,          // M (for FEM with non-lumped masses, add their mass-matrices)
                        1.0,          // Ct term
                        0,            // no C term
                        0.0,          //
                        false);       // no clamping on -C/dt

        // Make the vectors of pointers to constraint and variables, for LCP solver
        LCPprepare_inject(*this->LCP_descriptor);

        // Solve the LCP problem using the speed solver. Solution variables are new
        // speeds 'v_new'
        GetLcpSolverSpeed()->Solve(*this->LCP_descriptor);

        // Update the constraint reaction forces.
        LCPresult_Li_into_reactions(1 / step);

        // Loop over all bodies and other physics items; approximate speeds using
        // finite diferences (with the local, small time step value) and update all
        // markers and forces.
        for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
        {
            ChSharedPtr<ChBody> Bpointer = bodylist[ip];
            Bpointer->VariablesQbSetSpeed(step);
            Bpointer->Update(this->ChTime);
        }

        for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
        {
            ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];
            PHpointer->VariablesQbSetSpeed(step);
            PHpointer->Update(this->ChTime);
        }

        // Restore the time step and max number of iterations.
        step = step_saved;
        SetIterLCPmaxItersSpeed(niter_saved);
    }

    return 0;
}

// **** PERFORM THE LINEAR STATIC ANALYSIS

int ChSystem::DoStaticLinear() {
    
    this->solvecount = 0;
    
    Setup();
    Update();

    int old_maxsteps = this->GetIterLCPmaxItersSpeed();
    this->SetIterLCPmaxItersSpeed(300);

    // Prepare lists of variables and constraints. 
    LCPprepare_inject(*this->LCP_descriptor);

    ChStaticLinearAnalysis manalysis(*this);

    // Perform analysis
    manalysis.StaticAnalysis();

    this->SetIterLCPmaxItersSpeed(old_maxsteps);

    bool dump_data = false;

    if (dump_data) {
        this->GetLcpSystemDescriptor()->DumpLastMatrices();

        // optional check for correctness in result
        chrono::ChMatrixDynamic<double> md;
        GetLcpSystemDescriptor()->BuildDiVector(md);  // d={f;-b}

        chrono::ChMatrixDynamic<double> mx;
        GetLcpSystemDescriptor()->FromUnknownsToVector(mx);  // x ={q,-l}
        chrono::ChStreamOutAsciiFile file_x("dump_x.dat");
        mx.StreamOUTdenseMatlabFormat(file_x);

        chrono::ChMatrixDynamic<double> mZx;
        GetLcpSystemDescriptor()->SystemProduct(mZx, &mx);  // Zx = Z*x

        GetLog() << "CHECK: norm of solver residual: ||Z*x-d|| -------------------\n";
        GetLog() << (mZx - md).NormInf() << "\n";
    }

    return 0;
}

// **** PERFORM THE NONLINEAR STATIC ANALYSIS

int ChSystem::DoStaticNonlinear(int nsteps) {

    this->solvecount;

    Setup();
    Update();

    int old_maxsteps = this->GetIterLCPmaxItersSpeed();
    this->SetIterLCPmaxItersSpeed(300);

    // Prepare lists of variables and constraints. 
    LCPprepare_inject(*this->LCP_descriptor);

    ChStaticNonLinearAnalysis manalysis(*this);
    manalysis.SetMaxiters(nsteps);

    // Perform analysis
    manalysis.StaticAnalysis();

    this->SetIterLCPmaxItersSpeed(old_maxsteps);

    return 0;
}

// **** PERFORM THE STATIC ANALYSIS, FINDING THE STATIC
// **** EQUILIBRIUM OF THE SYSTEM, WITH ITERATIVE SOLUTION

int ChSystem::DoStaticRelaxing(int nsteps) {

    this->solvecount;

    int err = 0;
    int reached_tolerance = FALSE;


    if (ncoords > 0) {
        if (ndof >= 0) {
            for (int m_iter = 0; m_iter < nsteps; m_iter++) {
                for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
                {
                    ChSharedPtr<ChBody> Bpointer = bodylist[ip];
                    // Set no body speed and no body accel.
                    Bpointer->SetNoSpeedNoAcceleration();
                }
                for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
                {
                    ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];
                    PHpointer->SetNoSpeedNoAcceleration();
                }

                double m_undotime = this->GetChTime();
                DoFrameDynamics(m_undotime +
                                (step * 1.8) * (((double)nsteps - (double)m_iter)) /
                                    (double)nsteps);
                this->SetChTime(m_undotime);
            }

            for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
            {
                ChSharedPtr<ChBody> Bpointer = bodylist[ip];
                // Set no body speed and no body accel.
                Bpointer->SetNoSpeedNoAcceleration();
            }

            for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
            {
                ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];
                PHpointer->SetNoSpeedNoAcceleration();
            }
        }
    }

    if (err) {
        last_err = TRUE;
        GetLog() << "WARNING: some costraints may be redundant, but couldn't be eliminated \n";
    }
    return last_err;
}

// **** ---    THE KINEMATIC SIMULATION  ---
// **** PERFORM IK (INVERSE KINEMATICS) UNTIL THE END_TIME IS
// **** REACHED, STARTING FROM THE CURRENT TIME.

int ChSystem::DoEntireKinematics() {
    Setup();


    DoAssembly(ASS_POSITION | ASS_SPEED | ASS_ACCEL);
    // first check if there are redundant links (at least one NR cycle
    // even if the structure is already assembled)

    while (ChTime < end_time) {
        DoAssembly(ASS_POSITION | ASS_SPEED | ASS_ACCEL);  // >>> Newton-Raphson iteration, closing constraints

        if (last_err)
            return FALSE;

        ChTime += step;  // >>> Update the time and repeat.
    }

    return TRUE;
}

// **** ---   THE DYNAMICAL SIMULATION   ---
// **** PERFORM EXPLICIT OR IMPLICIT INTEGRATION TO GET
// **** THE DYNAMICAL SIMULATION OF THE SYSTEM, UNTIL THE
// **** END_TIME IS REACHED.

int ChSystem::DoEntireDynamics() {
    Setup();


    // the system may have wrong layout, or too large
    // clearances in costraints, so it is better to
    // check for costraint violation each time the integration starts
    DoAssembly(ASS_POSITION | ASS_SPEED | ASS_ACCEL);

    // Perform the integration steps until the end
    // time is reached.
    // All the updating (of Y, Y_dt and time) is done
    // automatically by Integrate()

    while (ChTime < end_time) {
        if (!Integrate_Y())
            break;  // >>> 1- single integration step,
                    //        updating Y, from t to t+dt.
        if (last_err)
            return FALSE;
    }

    if (last_err)
        return FALSE;
    return TRUE;
}



// Perform the dynamical integration, from current ChTime to
// the specified m_endtime, and terminating the integration exactly
// on the m_endtime. Therefore, the step of integration may get a
// little increment/decrement to have the last step ending in m_endtime.
// Note that this function can be used in iterations to provide results in
// a evenly spaced frames of time, even if the steps are changing.
// Also note that if the time step is higher than the time increment
// requested to reach m_endtime, the step is lowered.

int ChSystem::DoFrameDynamics(double m_endtime) {
    double frame_step;
    double old_step;
    double left_time;
    int restore_oldstep = FALSE;
    int counter = 0;
    double fixed_step_undo;

    frame_step = (m_endtime - ChTime);
    fixed_step_undo = step;

    while (ChTime < m_endtime) {
        restore_oldstep = FALSE;
        counter++;

        left_time = m_endtime - ChTime;

        if (left_time < 1e-12)
            break;  // - no integration if backward or null frame step.

        if (left_time < (1.3 * step))  // - step changed if too little frame step
        {
            old_step = step;
            step = left_time;
            restore_oldstep = TRUE;
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
        return FALSE;
    return TRUE;
}

// Performs the dynamical simulation, but using "frame integration"
// iteratively. The results are provided only at each frame (evenly
// spaced by "frame_step") rather than at each "step" (steps can be much
// more than frames, and they may be automatically changed by integrator).
// Moreover, the integration results shouldn't be dependent by the
// "frame_step" value (steps are performed anyway, like in normal "DoEntireDynamics"
// command).

int ChSystem::DoEntireUniformDynamics(double frame_step) {
    // the initial system may have wrong layout, or too large
    // clearances in constraints.
    Setup();
    DoAssembly(ASS_POSITION | ASS_SPEED | ASS_ACCEL);

    while (ChTime < end_time) {
        double goto_time = (ChTime + frame_step);
        if (!DoFrameDynamics(goto_time))
            return FALSE;  // ###### Perform "frame integration
    }

    return TRUE;
}

// Like DoFrameDynamics, but performs kinematics instead of dinamics

int ChSystem::DoFrameKinematics(double m_endtime) {
    double frame_step;
    double old_step;
    double left_time;
    int restore_oldstep;
    int counter = 0;

    frame_step = (m_endtime - ChTime);

    double fixed_step_undo = step;

    while (ChTime < m_endtime) {
        restore_oldstep = FALSE;
        counter++;

        left_time = m_endtime - ChTime;

        if (left_time < 0.000000001)
            break;  // - no kinematics for backward

        if (left_time < (1.3 * step))  // - step changed if too little frame step
        {
            old_step = step;
            step = left_time;
            restore_oldstep = TRUE;
        }

        DoAssembly(ASS_POSITION | ASS_SPEED | ASS_ACCEL);  // ***  Newton Raphson kinematic equations solver

        if (last_err)
            return FALSE;

        ChTime += step;

        if (restore_oldstep)
            step = old_step;  // if timestep was changed to meet the end of frametime
    }

    return TRUE;
}

int ChSystem::DoStepKinematics(double m_step) {

    ChTime += m_step;

    Update();

    DoAssembly(ASS_POSITION | ASS_SPEED | ASS_ACCEL);  // ***  Newton Raphson kinematic equations solver

    if (last_err)
        return FALSE;

    return TRUE;
}

//
// Full assembly -computes also forces-
//

int ChSystem::DoFullAssembly() {

    DoAssembly(ASS_POSITION | ASS_SPEED | ASS_ACCEL);

    return last_err;
}

////////
////////  STREAMING - FILE HANDLING
////////


void ChSystem::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

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

    eCh_lcpSolver_mapper msolmapper;
    marchive << CHNVP(msolmapper(lcp_solver_type),"lcp_solver_type");
    marchive << CHNVP(LCP_descriptor); 
    marchive << CHNVP(LCP_solver_speed); 
    marchive << CHNVP(LCP_solver_stab);  

    marchive << CHNVP(iterLCPmaxIters);
    marchive << CHNVP(iterLCPmaxItersStab);
    marchive << CHNVP(simplexLCPmaxSteps); 
    marchive << CHNVP(min_bounce_speed); 
    marchive << CHNVP(max_penetration_recovery_speed);
    marchive << CHNVP(parallel_thread_number); 

    marchive << CHNVP(collision_system);// ChCollisionSystem should implement class factory for abstract create

    //marchive << CHNVP(scriptEngine); // ChScriptEngine should implement class factory for abstract create
    marchive << CHNVP(scriptForStartFile);
    marchive << CHNVP(scriptForUpdateFile);
    marchive << CHNVP(scriptForStepFile);
    marchive << CHNVP(scriptFor3DStepFile);

    eCh_integrationType_mapper mintmapper;
    marchive << CHNVP(mintmapper(integration_type),"integration_type");
    marchive << CHNVP(timestepper); // ChTimestepper should implement class factory for abstract create

    //***TODO*** complete...
}

/// Method to allow de serialization of transient data from archives.
void ChSystem::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

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

    eCh_lcpSolver_mapper msolmapper;
    marchive >> CHNVP(msolmapper(lcp_solver_type),"lcp_solver_type");

    if (LCP_descriptor) delete LCP_descriptor;
    marchive >> CHNVP(LCP_descriptor); 

    if (LCP_solver_speed) delete LCP_solver_speed;
    marchive >> CHNVP(LCP_solver_speed); 
    
    if (LCP_solver_stab) delete LCP_solver_stab;
    marchive >> CHNVP(LCP_solver_stab);  

    marchive >> CHNVP(iterLCPmaxIters);
    marchive >> CHNVP(iterLCPmaxItersStab);
    marchive >> CHNVP(simplexLCPmaxSteps); 
    marchive >> CHNVP(min_bounce_speed); 
    marchive >> CHNVP(max_penetration_recovery_speed);
    marchive >> CHNVP(parallel_thread_number); 

    if (collision_system) delete collision_system;
    marchive >> CHNVP(collision_system);// ChCollisionSystem should implement class factory for abstract create

    //marchive >> CHNVP(scriptEngine); // ChScriptEngine should implement class factory for abstract create
    marchive >> CHNVP(scriptForStartFile);
    marchive >> CHNVP(scriptForUpdateFile);
    marchive >> CHNVP(scriptForStepFile);
    marchive >> CHNVP(scriptFor3DStepFile);

    eCh_integrationType_mapper mintmapper;
    marchive >> CHNVP(mintmapper(integration_type),"integration_type");

    marchive >> CHNVP(timestepper); // ChTimestepper should implement class factory for abstract create
    timestepper->SetIntegrable(this);

    //***TODO*** complete...

    //  Rebuild link pointers to markers
    this->Reference_LM_byID();

    // Recompute statistics, offsets, etc.
    this->Setup();
}





#define CH_CHUNK_START "Chrono binary file start"
#define CH_CHUNK_END "Chrono binary file end"

int ChSystem::FileProcessChR(ChStreamInBinary& m_file) {
    std::string mchunk;

    m_file >> mchunk;
    if (mchunk != CH_CHUNK_START)
        throw ChException("Not a ChR data file.");

    //this->StreamINall(m_file);

    m_file >> mchunk;
    if (mchunk != CH_CHUNK_END)
        throw ChException("The end of ChR data file is badly formatted.");

    return 1;
}

int ChSystem::FileWriteChR(ChStreamOutBinary& m_file) {
    m_file << CH_CHUNK_START;

    //this->StreamOUTall(m_file);

    m_file << CH_CHUNK_END;

    return 1;
}




}  // END_OF_NAMESPACE____

/////////////////////////////////////// eof
