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

///////////////////////////////////////////////////
//
//   ChSystem.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

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
//#include "lcp/ChLcpSolverDEM.h"
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
        mptr = 0;
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
                    mptr = 0;
                    return (*this);
                }
            }  // end cases
        } while (true);

        return (*this);
    }

    ChPhysicsItem* operator->() { return (mptr); }
    ChPhysicsItem* operator*() { return (mptr); }

  private:
    std::vector<ChBody*>::iterator node_body;
    std::vector<ChBody*>* list_bodies;
    std::vector<ChLink*>::iterator node_link;
    std::vector<ChLink*>* list_links;
    std::vector<ChPhysicsItem*>::iterator node_otherphysics;
    std::vector<ChPhysicsItem*>* list_otherphysics;
    ChPhysicsItem* mptr;
    int stage;
    ChSystem* msystem;
};

//////////////////////////////////////
//////////////////////////////////////
// CLASS FOR PHYSICAL SYSTEM

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChSystem> a_registration_ChSystem;

ChSystem::ChSystem(unsigned int max_objects, double scene_size, bool init_sys) {
    linklist.clear();
    bodylist.clear();
    otherphysicslist.clear();
    probelist.clear();
    controlslist.clear();

    nbodies = 0;
    nlinks = 0;
    nphysicsitems = 0;
    ndof = 0;
    ndoc = 0;
    ndoc_w = 0;
    ndoc_w_C = 0;
    ndoc_w_D = 0;
    nsysvars_w = 0;
    ncoords = 0;
    ncoords_w = 0;
    nsysvars = 0;
    ncoords_w = 0;
    ncontacts = 0;
    nbodies_sleep = 0;
    nbodies_fixed = 0;

    // ------ PREFERENCES INITIALIZATION :
    ChTime = 0;

    end_time = 1;
    step = 0.04;
    step_min = 0.002;
    step_max = 0.04;
    tol = 0.0002;
    tol_force = 1e-3;
    normtype = NORM_INF;
    maxiter = 6;

    SetIntegrationType(INT_EULER_IMPLICIT_LINEARIZED); 

    min_bounce_speed = 0.15;
    max_penetration_recovery_speed = 0.6;

    parallel_thread_number = CHOMPfunctions::GetNumProcs();  // default n.threads as n.cores

    this->contact_container = 0;
    // default contact container
    if (init_sys) {
        this->contact_container = new ChContactContainerDVI();
        this->contact_container->SetSystem(this);
    }
    collision_system = 0;
    // default GPU collision engine
    if (init_sys) {
        collision_system = new ChCollisionSystemBullet(max_objects, scene_size);
    }

    this->timestepper =
        ChSharedPtr<ChTimestepperEulerImplicitLinearized>(new ChTimestepperEulerImplicitLinearized(*this));  // OK

    LCP_descriptor = 0;
    LCP_solver_speed = 0;
    LCP_solver_stab = 0;

    iterLCPmaxIters = 30;
    iterLCPmaxItersStab = 10;
    
    if (init_sys) {
        SetLcpSolverType(LCP_ITERATIVE_SYMMSOR);
    }

    use_sleeping = false;

    collision_callback = 0;
    collisionpoint_callback = 0;

    Set_G_acc(ChVector<>(0, -9.8, 0));

    stepcount = 0;

    last_err = 0;

    scriptEngine = 0;
    scriptForStart = NULL;
    scriptForUpdate = NULL;
    scriptForStep = NULL;
    scriptFor3DStep = NULL;
    strcpy(scriptForStartFile, "");
    strcpy(scriptForUpdateFile, "");
    strcpy(scriptForStepFile, "");
    strcpy(scriptFor3DStepFile, "");

    events = new ChEvents(250);
}

ChSystem::~ChSystem() {
    RemoveAllBodies();
    RemoveAllLinks();
    RemoveAllOtherPhysicsItems();
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
    if (contact_container)
        delete contact_container;
    contact_container = 0;

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
    // first copy the parent class data...
    ChObj::Copy(source);

    G_acc = source->Get_G_acc();
    end_time = source->GetEndTime();
    step = source->GetStep();
    step_min = source->GetStepMin();
    step_max = source->GetStepMax();
    SetIntegrationType(source->GetIntegrationType());
    tol = source->GetTol();
    tol_force = source->tol_force;
    normtype = source->GetNormType();
    maxiter = source->GetMaxiter();
    nbodies = source->GetNbodies();
    nlinks = source->GetNlinks();
    nphysicsitems = source->GetNphysicsItems();
    ncoords = source->GetNcoords();
    ncoords_w = source->GetNcoords_w();
    ndoc = source->GetNdoc();
    ndoc_w = source->GetNdoc_w();
    ndoc_w_C = source->GetNdoc_w_C();
    ndoc_w_D = source->GetNdoc_w_D();
    ndof = source->GetNdof();
    nsysvars = source->GetNsysvars();
    nsysvars_w = source->GetNsysvars_w();
    ncontacts = source->GetNcontacts();
    nbodies_sleep = source->GetNbodiesSleeping();
    nbodies_fixed = source->GetNbodiesFixed();
    min_bounce_speed = source->min_bounce_speed;
    max_penetration_recovery_speed = source->max_penetration_recovery_speed;
    iterLCPmaxIters = source->iterLCPmaxIters;
    iterLCPmaxItersStab = source->iterLCPmaxItersStab;
    SetLcpSolverType(GetLcpSolverType());
    parallel_thread_number = source->parallel_thread_number;
    use_sleeping = source->use_sleeping;


    collision_callback = source->collision_callback;
    collisionpoint_callback = source->collisionpoint_callback;

    last_err = source->last_err;

    RemoveAllLinks();
    RemoveAllBodies();
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
    nbodies = 0;
    nlinks = 0;
    nphysicsitems = 0;
    ndof = 0;
    ndoc = 0;
    ndoc_w = 0;
    ndoc_w_C = 0;
    ndoc_w_D = 0;
    nsysvars_w = 0;
    ncoords = 0;
    ncoords_w = 0;
    nsysvars = 0;
    ncoords_w = 0;
    ncontacts = 0;
    nbodies_sleep = 0;
    nbodies_fixed = 0;

    events->ResetAllEvents();

    // contact_container->RemoveAllContacts();

    RemoveAllLinks();
    RemoveAllBodies();
    RemoveAllOtherPhysicsItems();
    RemoveAllProbes();
    RemoveAllControls();

    // ResetTimers();
}

//
// Set/Get routines
//

void ChSystem::SetLcpSolverType(eCh_lcpSolver mval) {
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
    if (contact_container)
        delete contact_container;
    contact_container = 0;

    LCP_descriptor = new ChLcpSystemDescriptor;
    LCP_descriptor->SetNumThreads(parallel_thread_number);

    this->contact_container = new ChContactContainerDVI();
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
}

void ChSystem::ChangeLcpSolverStab(ChLcpSolver* newsolver) {
    assert(newsolver);
    if (this->LCP_solver_stab)
        delete (this->LCP_solver_stab);
    this->LCP_solver_stab = newsolver;
}

void ChSystem::ChangeContactContainer(ChContactContainerBase* newcontainer) {
    assert(newcontainer);
    if (this->contact_container)
        delete (this->contact_container);
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

int ChSystem::SetScriptForStartFile(char* mfile) {
    if (!this->scriptEngine)
        return 0;
    strncpy(this->scriptForStartFile, mfile, sizeof(this->scriptForStartFile)-1);
    this->scriptForStart = this->scriptEngine->CreateScript();
    return this->scriptEngine->FileToScript(*this->scriptForStart, mfile);
}
int ChSystem::SetScriptForUpdateFile(char* mfile) {
    if (!this->scriptEngine)
        return 0;
    strncpy(this->scriptForUpdateFile, mfile, sizeof(this->scriptForUpdateFile)-1);
    this->scriptForUpdate = this->scriptEngine->CreateScript();
    return this->scriptEngine->FileToScript(*this->scriptForUpdate, mfile);
}
int ChSystem::SetScriptForStepFile(char* mfile) {
    if (!this->scriptEngine)
        return 0;
    strncpy(this->scriptForStepFile, mfile, sizeof(this->scriptForStepFile)-1);
    this->scriptForStep = this->scriptEngine->CreateScript();
    return this->scriptEngine->FileToScript(*this->scriptForStep, mfile);
}
int ChSystem::SetScriptFor3DStepFile(char* mfile) {
    if (!this->scriptEngine)
        return 0;
    strncpy(this->scriptFor3DStepFile, mfile, sizeof(this->scriptFor3DStepFile)-1);
    this->scriptFor3DStep = this->scriptEngine->CreateScript();
    return this->scriptEngine->FileToScript(*this->scriptFor3DStep, mfile);
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
        ChProbe* Ppointer = probelist[ip];

        Ppointer->Record(this->GetChTime());
    }

    return pcount;
}

int ChSystem::ResetAllProbes() {
    int pcount = 0;

    for (unsigned int ip = 0; ip < probelist.size(); ++ip)  // ITERATE on probes
    {
        ChProbe* Ppointer = probelist[ip];

        Ppointer->Reset();
    }

    return pcount;
}

// CONTROLS STUFF

int ChSystem::ExecuteControlsForStart() {
    for (unsigned int ip = 0; ip < controlslist.size(); ++ip)  // ITERATE on controls
    {
        ChControls* Cpointer = controlslist[ip];

        Cpointer->ExecuteForStart();
    }
    return TRUE;
}

int ChSystem::ExecuteControlsForUpdate() {
    for (unsigned int ip = 0; ip < controlslist.size(); ++ip)  // ITERATE on controls
    {
        ChControls* Cpointer = controlslist[ip];

        Cpointer->ExecuteForUpdate();
    }
    return TRUE;
}

int ChSystem::ExecuteControlsForStep() {
    for (unsigned int ip = 0; ip < controlslist.size(); ++ip)  // ITERATE on controls
    {
        ChControls* Cpointer = controlslist[ip];

        Cpointer->ExecuteForStep();
    }
    return TRUE;
}

int ChSystem::ExecuteControlsFor3DStep() {
    for (unsigned int ip = 0; ip < controlslist.size(); ++ip)  // ITERATE on controls
    {
        ChControls* Cpointer = controlslist[ip];

        Cpointer->ExecuteFor3DStep();
    }
    return TRUE;
}

//
// HIERARCHY HANDLERS
//

void ChSystem::AddBody(ChSharedPtr<ChBody> newbody) {
    assert(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), newbody.get_ptr()) ==
           bodylist.end());
    assert(newbody->GetSystem() == 0);  // should remove from other system before adding here

    newbody->AddRef();
    newbody->SetSystem(this);
    bodylist.push_back((newbody).get_ptr());

    // add to collision system too
    if (newbody->GetCollide())
        newbody->AddCollisionModelsToSystem();
}

void ChSystem::RemoveBody(ChSharedPtr<ChBody> mbody) {
    assert(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr()) !=
           bodylist.end());

    // remove from collision system
    if (mbody->GetCollide())
        mbody->RemoveCollisionModelsFromSystem();

    // warning! linear time search, to erase pointer from container.
    bodylist.erase(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr()));

    // nullify backward link to system
    mbody->SetSystem(0);
    // this may delete the body, if none else's still referencing it..
    mbody->RemoveRef();
}

void ChSystem::AddLink(ChLink* newlink) {
    assert(std::find<std::vector<ChLink*>::iterator>(linklist.begin(), linklist.end(), newlink) == linklist.end());

    newlink->AddRef();
    newlink->SetSystem(this);
    linklist.push_back(newlink);
}

void ChSystem::AddLink(ChSharedPtr<ChLink> newlink) {
    AddLink(newlink.get_ptr());
}

// Faster than RemoveLink because it does not require the linear time search
std::vector<ChLink*>::iterator ChSystem::RemoveLinkIter(std::vector<ChLink*>::iterator& mlinkiter) {
    // nullify backward link to system
    (*mlinkiter)->SetSystem(0);
    // this may delete the link, if none else's still referencing it..
    (*mlinkiter)->RemoveRef();

    return linklist.erase(mlinkiter);
}

void ChSystem::RemoveLink(ChSharedPtr<ChLink> mlink) {
    assert(std::find<std::vector<ChLink*>::iterator>(linklist.begin(), linklist.end(), mlink.get_ptr()) !=
           linklist.end());

    // warning! linear time search, to erase pointer from container!
    linklist.erase(std::find<std::vector<ChLink*>::iterator>(linklist.begin(), linklist.end(), mlink.get_ptr()));

    // nullify backward link to system
    mlink->SetSystem(0);
    // this may delete the body, if none else's still referencing it..
    mlink->RemoveRef();
}

void ChSystem::AddOtherPhysicsItem(ChSharedPtr<ChPhysicsItem> newitem) {
    assert(std::find<std::vector<ChPhysicsItem*>::iterator>(otherphysicslist.begin(), otherphysicslist.end(),
                                                            newitem.get_ptr()) == otherphysicslist.end());
    // assert(newitem->GetSystem()==0); // should remove from other system before adding here

    newitem->AddRef();
    newitem->SetSystem(this);
    otherphysicslist.push_back((newitem).get_ptr());

    // add to collision system too
    if (newitem->GetCollide())
        newitem->AddCollisionModelsToSystem();
}

void ChSystem::RemoveOtherPhysicsItem(ChSharedPtr<ChPhysicsItem> mitem) {
    assert(std::find<std::vector<ChPhysicsItem*>::iterator>(otherphysicslist.begin(), otherphysicslist.end(),
                                                            mitem.get_ptr()) != otherphysicslist.end());

    // remove from collision system
    if (mitem->GetCollide())
        mitem->RemoveCollisionModelsFromSystem();

    // warning! linear time search, to erase pointer from container.
    otherphysicslist.erase(std::find<std::vector<ChPhysicsItem*>::iterator>(otherphysicslist.begin(),
                                                                            otherphysicslist.end(), mitem.get_ptr()));

    // nullify backward link to system
    mitem->SetSystem(0);
    // this may delete the body, if none else's still referencing it..
    mitem->RemoveRef();
}

void ChSystem::Add(ChSharedPtr<ChPhysicsItem> newitem) {
    if (newitem.IsType<ChBody>()) // (typeid(*newitem.get_ptr())==typeid(ChBody)) // if (newitem.IsType<ChBody>()) sends ChBody descendants in ChBody list: this is bad for ChConveyor
    {
        AddBody(newitem.DynamicCastTo<ChBody>());
    } else if (newitem.IsType<ChLink>()) {
        AddLink(newitem.DynamicCastTo<ChLink>());
    } else
        AddOtherPhysicsItem(newitem);
}

void ChSystem::AddBatch(ChSharedPtr<ChPhysicsItem> newitem) {
    // the following is a openMP critical section:
    #pragma omp critical
    {
        this->batch_to_insert.push_back(newitem);
        newitem->SetSystem(this);
    }
}
void ChSystem::FlushBatch() {
    for (int i=0; i<  this->batch_to_insert.size(); ++i) {
        batch_to_insert[i]->SetSystem(0);
        this->Add(batch_to_insert[i]);
    }
    batch_to_insert.clear();
}

void ChSystem::Remove(ChSharedPtr<ChPhysicsItem> newitem) {
    if (newitem.IsType<ChBody>()) // (typeid(*newitem.get_ptr())==typeid(ChBody)) // if (newitem.IsType<ChBody>()) sends ChBody descendants in ChBody list: this is bad for ChConveyor
    {
        RemoveBody(newitem.DynamicCastTo<ChBody>());
    } else if (newitem.IsType<ChLink>()) {
        RemoveLink(newitem.DynamicCastTo<ChLink>());
    } else
        RemoveOtherPhysicsItem(newitem);
}

void ChSystem::AddProbe(ChSharedPtr<ChProbe>& newprobe) {
    assert(std::find<std::vector<ChProbe*>::iterator>(probelist.begin(), probelist.end(), newprobe.get_ptr()) ==
           probelist.end());

    newprobe->AddRef();
    // newprobe->SetSystem (this);
    probelist.push_back(newprobe.get_ptr());
}

void ChSystem::AddControls(ChSharedPtr<ChControls>& newcontrols) {
    assert(std::find<std::vector<ChControls*>::iterator>(controlslist.begin(), controlslist.end(),
                                                         newcontrols.get_ptr()) == controlslist.end());

    newcontrols->AddRef();
    // newcontrols->SetSystem (this);
    controlslist.push_back(newcontrols.get_ptr());
}

void ChSystem::RemoveAllBodies() {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];

        // remove from collision system
        if (Bpointer->GetCollide())
            Bpointer->RemoveCollisionModelsFromSystem();
        // nullify backward link to system
        Bpointer->SetSystem(0);
        // this may delete the body, if none else's still referencing it..
        Bpointer->RemoveRef();
    }
    bodylist.clear();
}

void ChSystem::RemoveAllLinks() {
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];

        // nullify backward link to system
        Lpointer->SetSystem(0);
        // this may delete the link, if none else's still referencing it..
        Lpointer->RemoveRef();
    }
    linklist.clear();
}

void ChSystem::RemoveAllOtherPhysicsItems() {
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];

        // remove from collision system
        if (PHpointer->GetCollide())
            PHpointer->RemoveCollisionModelsFromSystem();
        // nullify backward link to system
        PHpointer->SetSystem(0);
        // this may delete the item, if none else's still referencing it..
        PHpointer->RemoveRef();
    }
    otherphysicslist.clear();
}

void ChSystem::RemoveAllProbes() {
    for (unsigned int ip = 0; ip < probelist.size(); ++ip)  // ITERATE on probes
    {
        ChProbe* Ppointer = probelist[ip];

        Ppointer->RemoveRef();
    }
    probelist.clear();
}

void ChSystem::RemoveAllControls() {
    for (unsigned int ip = 0; ip < controlslist.size(); ++ip)  // ITERATE on controls
    {
        ChControls* Cpointer = controlslist[ip];

        // Cpointer->SetSystem(0);
        Cpointer->RemoveRef();
    }
    controlslist.clear();
}

ChSharedPtr<ChBody> ChSystem::SearchBody(const char* m_name) {
    ChBody* mbody =
        ChContainerSearchFromName<ChBody, std::vector<ChBody*>::iterator>(m_name, bodylist.begin(), bodylist.end());
    if (mbody) {
        mbody->AddRef();  // in that container pointers were not stored as ChSharedPtr, so this is needed..
        return (ChSharedPtr<ChBody>(
            mbody));  // ..here I am not getting a new() data, but a reference to something created elsewhere
    }
    return (ChSharedPtr<ChBody>());  // not found? return a void shared ptr.
}

ChSharedPtr<ChLink> ChSystem::SearchLink(const char* m_name) {
    ChLink* mlink =
        ChContainerSearchFromName<ChLink, std::vector<ChLink*>::iterator>(m_name, linklist.begin(), linklist.end());
    if (mlink) {
        mlink->AddRef();  // in that container pointers were not stored as ChSharedPtr, so this is needed..
        return (ChSharedPtr<ChLink>(
            mlink));  // ..here I am not getting a new() data, but a reference to something created elsewhere
    }
    return (ChSharedPtr<ChLink>());  // not found? return a void shared ptr.
}

ChSharedPtr<ChPhysicsItem> ChSystem::SearchOtherPhysicsItem(const char* m_name) {
    ChPhysicsItem* mitem = ChContainerSearchFromName<ChPhysicsItem, std::vector<ChPhysicsItem*>::iterator>(
        m_name, otherphysicslist.begin(), otherphysicslist.end());
    if (mitem) {
        mitem->AddRef();  // in that container pointers were not stored as ChSharedPtr, so this is needed..
        return (ChSharedPtr<ChPhysicsItem>(
            mitem));  // ..here I am not getting a new() data, but a reference to something created elsewhere
    }
    return (ChSharedPtr<ChPhysicsItem>());  // not found? return a void shared ptr.
}

ChSharedPtr<ChPhysicsItem> ChSystem::Search(const char* m_name) {
    ChSharedPtr<ChBody> mbo = SearchBody(m_name);
    if (!mbo.IsNull())
        return mbo;
    ChSharedPtr<ChLink> mli = SearchLink(m_name);
    if (!mli.IsNull())
        return mli;
    ChSharedPtr<ChPhysicsItem> mph = SearchOtherPhysicsItem(m_name);
    if (!mph.IsNull())
        return mph;
    return (ChSharedPtr<ChPhysicsItem>());  // not found? return a void shared ptr.
}

ChSharedPtr<ChMarker> ChSystem::SearchMarker(const char* m_name) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];

        ChSharedPtr<ChMarker> mmark = Bpointer->SearchMarker(m_name);
        if (!mmark.IsNull())
            return mmark;
    }

    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];

        if (ChBodyAuxRef* mbodyauxref = dynamic_cast<ChBodyAuxRef*>(PHpointer)) {
            ChSharedPtr<ChMarker> mmark = mbodyauxref->SearchMarker(m_name);
            if (!mmark.IsNull())
                return mmark;
        }
    }

    return (ChSharedPtr<ChMarker>());  // not found? return a void shared ptr.
}

ChSharedPtr<ChMarker> ChSystem::SearchMarker(int markID) {
    ChMarker* candidate = NULL;
    ChMarker* res = NULL;

    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];

        res = ChContainerSearchFromID<ChMarker, std::vector<ChMarker*>::const_iterator>(
            markID, Bpointer->GetMarkerList().begin(), Bpointer->GetMarkerList().end());
        if (res != NULL) {
            res->AddRef();  // in that container pointers were not stored as ChSharedPtr, so this is needed..
            return (ChSharedPtr<ChMarker>(
                res));  // ..here I am not getting a new() data, but a reference to something created elsewhere
        }
    }

    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];

        if (ChBodyAuxRef* mbodyauxref = dynamic_cast<ChBodyAuxRef*>(PHpointer)) {
            res = ChContainerSearchFromID<ChMarker, std::vector<ChMarker*>::const_iterator>(
                markID, mbodyauxref->GetMarkerList().begin(), mbodyauxref->GetMarkerList().end());
            if (res != NULL) {
                res->AddRef();  // in that container pointers were not stored as ChSharedPtr, so this is needed..
                return (ChSharedPtr<ChMarker>(
                    res));  // ..here I am not getting a new() data, but a reference to something created elsewhere
            }
        }
    }

    return (ChSharedPtr<ChMarker>());  // not found? return a void shared ptr.
}

void ChSystem::Reference_LM_byID() {
    std::vector<ChLink*> toremove;

    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];

        if (ChLinkMarkers* malink = ChDynamicCast(ChLinkMarkers, Lpointer)) {
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
        ChSharedPtr<ChLink> mlink(toremove[ir]);
        toremove[ir]->AddRef();  // cause shared from raw pointer from vector cointainer

        RemoveLink(mlink);
    }
}

//////
////// PREFERENCES

void ChSystem::SetIntegrationType(eCh_integrationType m_integration) {
    if (m_integration == integration_type)
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
            this->timestepper = ChSharedPtr<ChTimestepperEulerImplicit>(new ChTimestepperEulerImplicit(*this));
            (this->timestepper.DynamicCastTo<ChTimestepperEulerImplicit>())->SetMaxiters(4);
            break;
        case INT_EULER_IMPLICIT_LINEARIZED:
            this->timestepper =
                ChSharedPtr<ChTimestepperEulerImplicitLinearized>(new ChTimestepperEulerImplicitLinearized(*this));
            break;
        case INT_EULER_IMPLICIT_PROJECTED:
            this->timestepper =
                ChSharedPtr<ChTimestepperEulerImplicitProjected>(new ChTimestepperEulerImplicitProjected(*this));
            break;
        case INT_TRAPEZOIDAL:
            this->timestepper = ChSharedPtr<ChTimestepperTrapezoidal>(new ChTimestepperTrapezoidal(*this));
            (this->timestepper.DynamicCastTo<ChTimestepperTrapezoidal>())->SetMaxiters(4);
            break;
        case INT_TRAPEZOIDAL_LINEARIZED:
            this->timestepper =
                ChSharedPtr<ChTimestepperTrapezoidalLinearized>(new ChTimestepperTrapezoidalLinearized(*this));
            (this->timestepper.DynamicCastTo<ChTimestepperTrapezoidalLinearized>())->SetMaxiters(4);
            break;
        case INT_HHT:
            this->timestepper = ChSharedPtr<ChTimestepperHHT>(new ChTimestepperHHT(*this));
            (this->timestepper.DynamicCastTo<ChTimestepperHHT>())->SetMaxiters(4);
            break;
        case INT_HEUN:
            this->timestepper = ChSharedPtr<ChTimestepperHeun>(new ChTimestepperHeun(*this));
            break;
        case INT_RUNGEKUTTA45:
            this->timestepper = ChSharedPtr<ChTimestepperRungeKuttaExpl>(new ChTimestepperRungeKuttaExpl(*this));
            break;
        case INT_EULER_EXPLICIT:
            this->timestepper = ChSharedPtr<ChTimestepperEulerExplIIorder>(new ChTimestepperEulerExplIIorder(*this));
            break;
        case INT_LEAPFROG:
            this->timestepper = ChSharedPtr<ChTimestepperLeapfrog>(new ChTimestepperLeapfrog(*this));
            break;
        case INT_NEWMARK:
            this->timestepper = ChSharedPtr<ChTimestepperNewmark>(new ChTimestepperNewmark(*this));
            break;
        default:
            throw ChException("SetIntegrationType: timestepper not supported");
    }
}

void ChSystem::WakeUpSleepingBodies() {
    // Make this class for iterating through contacts (if supported by
    // contact container)

    class _wakeup_reporter_class : public ChReportContactCallback {
      public:
        /// Callback, used to report contact points already added to the container.
        /// This must be implemented by a child class of ChReportContactCallback.
        /// If returns false, the contact scanning will be stopped.
        virtual bool ReportContactCallback(
            const ChVector<>& pA,             ///< get contact pA
            const ChVector<>& pB,             ///< get contact pB
            const ChMatrix33<>& plane_coord,  ///< get contact plane coordsystem (A column 'X' is contact normal)
            const double& distance,           ///< get contact distance
            const float& mfriction,           ///< get friction info
            const ChVector<>& react_forces,   ///< get react.forces (if already computed). In coordsystem 'plane_coord'
            const ChVector<>& react_torques,  ///< get react.torques, if rolling friction (if already computed)
            collision::ChCollisionModel*
                modA,  ///< get model A (note: some containers may not support it and could be zero!)
            collision::ChCollisionModel*
                modB  ///< get model B (note: some containers may not support it and could be zero!)
            ) {
            if (!(modA && modB))
                return true;
            ChBody* b1 = dynamic_cast<ChBody*>(modA->GetContactable());
            ChBody* b2 = dynamic_cast<ChBody*>(modB->GetContactable());
            if (!(b1 && b2))
                return true;
            bool sleep1 = b1->GetSleeping();
            bool sleep2 = b2->GetSleeping();
            bool ground1 = b1->GetBodyFixed();
            bool ground2 = b2->GetBodyFixed();
            if (sleep1 && !sleep2 && !ground2) {
                b1->SetSleeping(false);
            }
            if (sleep2 && !sleep1 && !ground1) {
                b2->SetSleeping(false);
            }
            this->someone_sleeps = sleep1 | sleep2 | this->someone_sleeps;

            return true;  // to continue scanning contacts
        }

        // Data
        bool someone_sleeps;
    };

    _wakeup_reporter_class my_waker;

    if (this->GetUseSleeping()) {
        for (int i = 0; i < 1; i++)  //***TO DO*** reconfigurable number of wakeup cycles
        {
            my_waker.someone_sleeps = false;

            // scan all links and wake connected bodies
            for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
            {
                ChLink* Lpointer = linklist[ip];

                if (Lpointer->IsRequiringWaking()) {
                    ((ChBody*)Lpointer->GetBody1())->SetSleeping(false);
                    ((ChBody*)Lpointer->GetBody2())->SetSleeping(false);
                }
            }

            // scan all contacts and wake neighbouring bodies
            this->contact_container->ReportAllContacts(&my_waker);

            // bailout wakeup cycle prematurely, if all bodies are not sleeping
            if (!my_waker.someone_sleeps)
                break;
        }
    }
}

////////////////////////////////
//////
////// UPDATING ROUTINES
//////
//////

// COUNT ALL BODIES AND LINKS, ETC, COMPUTE &SET DOF FOR STATISTICS,
// ALLOCATES OR REALLOCATE BOOKKEEPING DATA/VECTORS, IF ANY

void ChSystem::Setup() {
    events->Record(CHEVENT_SETUP);

    nbodies = 0;
    nbodies_sleep = 0;
    nbodies_fixed = 0;
    ncoords = 0;
    ncoords_w = 0;
    ndoc = 0;
    ndoc_w = 0;
    ndoc_w_C = 0;
    ndoc_w_D = 0;
    nlinks = 0;
    nphysicsitems = 0;

    // Any item being queued for insertion in system's lists? add it.
    this->FlushBatch();

    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];

        if (Bpointer->GetBodyFixed())
            nbodies_fixed++;
        else if (Bpointer->GetSleeping())
            nbodies_sleep++;
        else {
            nbodies++;

            Bpointer->SetOffset_x(ncoords);
            Bpointer->SetOffset_w(ncoords_w);
            Bpointer->SetOffset_L(ndoc_w);

            // Bpointer->Setup(); // unneded since in bodies does nothing

            ncoords   += Bpointer->GetDOF();
            ncoords_w += Bpointer->GetDOF_w();
            ndoc_w    += Bpointer->GetDOC();   // unneeded since ChBody introduces no constraints
            ndoc_w_C  += Bpointer->GetDOC_c(); // unneeded since ChBody introduces no constraints
            ndoc_w_D  += Bpointer->GetDOC_d(); // unneeded since ChBody introduces no constraints
        }
    }

    ndoc += nbodies;  // add one quaternion constr. for each active body.

    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];

        nphysicsitems++;

        PHpointer->SetOffset_x(ncoords);
        PHpointer->SetOffset_w(ncoords_w);
        PHpointer->SetOffset_L(ndoc_w);

        PHpointer->Setup();  // compute DOFs etc. and sets the offsets also in child items, if assembly-type or
                             // mesh-type stuff

        ncoords   += PHpointer->GetDOF();
        ncoords_w += PHpointer->GetDOF_w();
        ndoc_w    += PHpointer->GetDOC();
        ndoc_w_C  += PHpointer->GetDOC_c();
        ndoc_w_D  += PHpointer->GetDOC_d();
    }

    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];

        if (Lpointer->IsActive()) {
            nlinks++;

            Lpointer->SetOffset_x(ncoords);
            Lpointer->SetOffset_w(ncoords_w);
            Lpointer->SetOffset_L(ndoc_w);

            Lpointer->Setup();  // compute DOFs etc. and sets the offsets also in child items, if any

            ncoords   += Lpointer->GetDOF();
            ncoords_w += Lpointer->GetDOF_w();
            ndoc_w    += Lpointer->GetDOC();
            ndoc_w_C  += Lpointer->GetDOC_c();
            ndoc_w_D  += Lpointer->GetDOC_d();
        }
    }

    {
        contact_container->SetOffset_L(ndoc_w);

        ndoc_w   += contact_container->GetDOC();
        ndoc_w_C += contact_container->GetDOC_c();
        ndoc_w_D += contact_container->GetDOC_d();
    }

    ndoc = ndoc_w + nbodies;          // number of constraints including quaternion constraints.
    nsysvars = ncoords + ndoc;        // total number of variables (coordinates + lagrangian multipliers)
    nsysvars_w = ncoords_w + ndoc_w;  // total number of variables (with 6 dof per body)

    ndof = ncoords - ndoc;  // number of degrees of freedom (approximate - does not consider constr. redundancy, etc)
}

// - ALL PHYSICAL ITEMS (BODIES, LINKS,ETC.) ARE UPDATED,
//   ALSO UPDATING THEIR AUXILIARY VARIABLES (ROT.MATRICES, ETC.).
// - UPDATES ALL FORCES  (AUTOMATIC, AS CHILDREN OF BODIES)
// - UPDATES ALL MARKERS (AUTOMATIC, AS CHILDREN OF BODIES).

void ChSystem::Update(bool update_assets) {

    timer_update.start();  // Timer for profiling

    events->Record(CHEVENT_UPDATE);  // Record an update event

    // Executes the "forUpdate" script, if any
    ExecuteScriptForUpdate();
    // Executes the "forUpdate" script
    // in all controls of controlslist
    ExecuteControlsForUpdate();

// --------------------------------------
// Spread state vector Y to bodies
//    Y --> Bodies
//    Y_accel --> Bodies
// Updates recursively all other aux.vars
// --------------------------------------
#pragma omp parallel for
    for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];

        Bpointer->Update(ChTime, update_assets);

        if (this->GetUseSleeping())
            Bpointer->TrySleeping();
    }
    // -----------------------------
    // Updates other physical items
    // -----------------------------
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];

        PHpointer->Update(ChTime, update_assets);
    }
    // -----------------------------
    // Updates all links
    // -----------------------------
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];

        Lpointer->Update(ChTime, update_assets);
    }

    this->contact_container->Update(ChTime, update_assets);  // Update all contacts, if any

    timer_update.stop();
}

///////////////////////////////
/////////
/////////   SPARSE LCP BOOKKEEPING
/////////

void ChSystem::LCPprepare_reset() {
#pragma omp parallel for
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        Lpointer->ConstraintsBiReset();
    }
#pragma omp parallel for
    for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        Bpointer->VariablesFbReset();
    }
#pragma omp parallel for
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
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
#pragma omp parallel for
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];

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
#pragma omp critical
            {
                Lpointer->ConstraintsFbLoadForces(F_factor);  // f*dt
            }
        }
    }

#pragma omp parallel for
    for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
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
    //   if it's worth it.  In the meantime, the loop is processed sequentially.

    ////#pragma omp parallel for
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
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
            ////#pragma omp critical
            ////{
            PHpointer->ConstraintsFbLoadForces(F_factor);  // f*dt
                                                           ////}
        }
    }

    if (C_factor)
        contact_container->ConstraintsBiLoad_C(C_factor, recovery_clamp, do_clamp);
    if (F_factor)
        contact_container->ConstraintsFbLoadForces(F_factor);  // f*dt
    if (load_jacobians)
        contact_container->ConstraintsLoadJacobians();
}

void ChSystem::LCPprepare_inject(ChLcpSystemDescriptor& mdescriptor) {
    mdescriptor.BeginInsertion();  // This resets the vectors of constr. and var. pointers.

    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        Lpointer->InjectConstraints(mdescriptor);
    }
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        Bpointer->InjectVariables(mdescriptor);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->InjectVariables(mdescriptor);
        PHpointer->InjectConstraints(mdescriptor);
        PHpointer->InjectKRMmatrices(mdescriptor);
    }
    this->contact_container->InjectConstraints(mdescriptor);

    mdescriptor.EndInsertion();
}

void ChSystem::LCPprepare_Li_from_speed_cache() {
#pragma omp parallel for
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        Lpointer->ConstraintsLiLoadSuggestedSpeedSolution();
    }
#pragma omp parallel for
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->ConstraintsLiLoadSuggestedSpeedSolution();
    }
    this->contact_container->ConstraintsLiLoadSuggestedSpeedSolution();
}

void ChSystem::LCPprepare_Li_from_position_cache() {
#pragma omp parallel for
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        Lpointer->ConstraintsLiLoadSuggestedPositionSolution();
    }
#pragma omp parallel for
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->ConstraintsLiLoadSuggestedPositionSolution();
    }
    this->contact_container->ConstraintsLiLoadSuggestedPositionSolution();
}

void ChSystem::LCPresult_Li_into_speed_cache() {
#pragma omp parallel for
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        Lpointer->ConstraintsLiFetchSuggestedSpeedSolution();
    }
#pragma omp parallel for
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->ConstraintsLiFetchSuggestedSpeedSolution();
    }
    this->contact_container->ConstraintsLiFetchSuggestedSpeedSolution();
}

void ChSystem::LCPresult_Li_into_position_cache() {
#pragma omp parallel for
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        Lpointer->ConstraintsLiFetchSuggestedPositionSolution();
    }
#pragma omp parallel for
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->ConstraintsLiFetchSuggestedPositionSolution();
    }
    this->contact_container->ConstraintsLiFetchSuggestedPositionSolution();
}

void ChSystem::LCPresult_Li_into_reactions(double mfactor) {
#pragma omp parallel for
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        Lpointer->ConstraintsFetch_react(mfactor);
    }
#pragma omp parallel for
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->ConstraintsFetch_react(mfactor);
    }
    this->contact_container->ConstraintsFetch_react(mfactor);
}

//////////////////////////////////
////////
////////    TIMESTEPPER INTERFACE
////////

/// From system to state y={x,v}
void ChSystem::StateGather(ChState& x, ChStateDelta& v, double& T) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntStateGather(Bpointer->GetOffset_x(), x, Bpointer->GetOffset_w(), v, T);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntStateGather(PHpointer->GetOffset_x(), x, PHpointer->GetOffset_w(), v, T);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntStateGather(Lpointer->GetOffset_x(), x, Lpointer->GetOffset_w(), v, T);
    }
    this->contact_container->IntStateGather(contact_container->GetOffset_x(), x, contact_container->GetOffset_w(), v, T); //  does nothing, but just in case implemented in future.
    T = this->GetChTime();
}

/// From state Y={x,v} to system.
void ChSystem::StateScatter(const ChState& x, const ChStateDelta& v, const double T) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntStateScatter(Bpointer->GetOffset_x(), x, Bpointer->GetOffset_w(), v, T);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntStateScatter(PHpointer->GetOffset_x(), x, PHpointer->GetOffset_w(), v, T);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntStateScatter(Lpointer->GetOffset_x(), x, Lpointer->GetOffset_w(), v, T);
    }
    this->contact_container->IntStateScatter(contact_container->GetOffset_x(), x, contact_container->GetOffset_w(), v, T); //  does nothing, but just in case implemented in future.

    this->SetChTime(T);
    this->Update();  //***TODO*** optimize because maybe IntStateScatter above might have already called Update?
}

/// From system to state derivative (acceleration), some timesteppers might need last computed accel.
void ChSystem::StateGatherAcceleration(ChStateDelta& a) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntStateGatherAcceleration(Bpointer->GetOffset_w(), a);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntStateGatherAcceleration(PHpointer->GetOffset_w(), a);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntStateGatherAcceleration(Lpointer->GetOffset_w(), a);
    }
    this->contact_container->IntStateGatherAcceleration(contact_container->GetOffset_w(), a); //  does nothing, but just in case implemented in future.
}

/// From state derivative (acceleration) to system, sometimes might be needed
void ChSystem::StateScatterAcceleration(const ChStateDelta& a) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntStateScatterAcceleration(Bpointer->GetOffset_w(), a);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntStateScatterAcceleration(PHpointer->GetOffset_w(), a);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntStateScatterAcceleration(Lpointer->GetOffset_w(), a);
    }
    this->contact_container->IntStateScatterAcceleration(contact_container->GetOffset_w(), a); //  does nothing, but just in case implemented in future.
}

/// From system to reaction forces (last computed) - some timestepper might need this
void ChSystem::StateGatherReactions(ChVectorDynamic<>& L) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntStateGatherReactions(Bpointer->GetOffset_L(), L);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntStateGatherReactions(PHpointer->GetOffset_L(), L);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntStateGatherReactions(Lpointer->GetOffset_L(), L);
    }
    this->contact_container->IntStateGatherReactions(contact_container->GetOffset_L(), L);
}

/// From reaction forces to system, ex. store last computed reactions in ChLink objects for plotting etc.
void ChSystem::StateScatterReactions(const ChVectorDynamic<>& L) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntStateScatterReactions(Bpointer->GetOffset_L(), L);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntStateScatterReactions(PHpointer->GetOffset_L(), L);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntStateScatterReactions(Lpointer->GetOffset_L(), L);
    }
    this->contact_container->IntStateScatterReactions(contact_container->GetOffset_L(), L);
}

/// Perform x_new = x + dx    for x in    Y = {x, dx/dt}
/// It takes care of the fact that x has quaternions, dx has angular vel etc.
/// NOTE: the system is not updated automatically after the state increment, so one might
/// need to call StateScatter() if needed.
void ChSystem::StateIncrementX(ChState& x_new,         ///< resulting x_new = x + Dx
                               const ChState& x,       ///< initial state x
                               const ChStateDelta& Dx  ///< state increment Dx
                               ) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntStateIncrement(Bpointer->GetOffset_x(), x_new, x, Bpointer->GetOffset_w(), Dx);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntStateIncrement(PHpointer->GetOffset_x(), x_new, x, PHpointer->GetOffset_w(), Dx);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntStateIncrement(Lpointer->GetOffset_x(), x_new, x, Lpointer->GetOffset_w(), Dx);
    }
    this->contact_container->IntStateIncrement(contact_container->GetOffset_x(), x_new, x, contact_container->GetOffset_w(), Dx); //  does nothing, but just in case implemented in future.
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
    if (force_state_scatter)
        this->StateScatter(x, v, T);

    // R and Qc vectors  --> LCP sparse solver structures  (also sets L and Dv to warmstart)

    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntToLCP(Bpointer->GetOffset_w(), Dv, R, Bpointer->GetOffset_L(), L, Qc);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntToLCP(PHpointer->GetOffset_w(), Dv, R, PHpointer->GetOffset_L(), L, Qc);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntToLCP(Lpointer->GetOffset_w(), Dv, R, Lpointer->GetOffset_L(), L, Qc);
    }
    this->contact_container->IntToLCP(contact_container->GetOffset_w(), Dv, R, contact_container->GetOffset_L(), L, Qc);

    // G and Cq  matrices:  fill the LCP sparse solver structures

    //#pragma omp parallel for
    for (int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];

        Lpointer->ConstraintsLoadJacobians();
    }
    //#pragma omp parallel for
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];

        PHpointer->ConstraintsLoadJacobians();

        if (c_a || c_v || c_x)
            PHpointer->KRMmatricesLoad(-c_x, -c_v, c_a);
    }

    contact_container->ConstraintsLoadJacobians();

    // prepare lists of variables and constraints, used by the following LCP solver

    LCPprepare_inject(*this->LCP_descriptor);

    bool dump_data = false;

    if (dump_data) {
        const char* numformat = "%.12g";
        // GetLog() << "StateSolveCorrection R=" << R << "\n\n";
        // GetLog() << "StateSolveCorrection Qc="<< Qc << "\n\n";
        // GetLog() << "StateSolveCorrection X=" << x << "\n\n";
        // GetLog() << "StateSolveCorrection V=" << v << "\n\n";

        this->LCP_descriptor->DumpLastMatrices("intpre_");

        chrono::ChStreamOutAsciiFile file_x("intpre_x.dat");
        file_x.SetNumFormat(numformat);
        ((ChMatrix<>)x).StreamOUTdenseMatlabFormat(file_x);

        chrono::ChStreamOutAsciiFile file_v("intpre_v.dat");
        file_v.SetNumFormat(numformat);
        ((ChMatrix<>)v).StreamOUTdenseMatlabFormat(file_v);

        chrono::ChStreamOutAsciiFile file_R("intpre_R.dat");
        file_R.SetNumFormat(numformat);
        ((ChMatrix<>)R).StreamOUTdenseMatlabFormat(file_R);

        chrono::ChStreamOutAsciiFile file_Qc("intpre_Qc.dat");
        file_Qc.SetNumFormat(numformat);
        ((ChMatrix<>)Qc).StreamOUTdenseMatlabFormat(file_Qc);
    }

    // Solve the LCP problem!!!

    timer_lcp.start();

    GetLcpSolverSpeed()->Solve(*this->LCP_descriptor);

    timer_lcp.stop();

    // Dv and L vectors  <-- LCP sparse solver structures

    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntFromLCP(Bpointer->GetOffset_w(), Dv, Bpointer->GetOffset_L(), L);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntFromLCP(PHpointer->GetOffset_w(), Dv, PHpointer->GetOffset_L(), L);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntFromLCP(Lpointer->GetOffset_w(), Dv, Lpointer->GetOffset_L(), L);
    }
    this->contact_container->IntFromLCP(contact_container->GetOffset_w(), Dv, contact_container->GetOffset_L(), L);

    if (dump_data) {
        const char* numformat = "%.12g";

        chrono::ChStreamOutAsciiFile file_Dv("intpost_Dv.dat");
        file_Dv.SetNumFormat(numformat);
        ((ChMatrix<>)Dv).StreamOUTdenseMatlabFormat(file_Dv);

        chrono::ChStreamOutAsciiFile file_L("intpost_L.dat");
        file_L.SetNumFormat(numformat);
        ((ChMatrix<>)L).StreamOUTdenseMatlabFormat(file_L);

        // GetLog() << "StateSolveCorrection Dv=" << Dv << "\n\n";
        // GetLog() << "StateSolveCorrection L="  << L << "\n\n";
        // GetLog() << "StateSolveCorrection Xn=" << x << "\n\n";
        // GetLog() << "StateSolveCorrection Vn=" << v << "\n\n";
    }
}

/// Increment a vector R with the term c*F:
///    R += c*F
void ChSystem::LoadResidual_F(ChVectorDynamic<>& R,  ///< result: the R residual, R += c*F
                              const double c         ///< a scaling factor
                              ) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntLoadResidual_F(Bpointer->GetOffset_w(), R, c);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntLoadResidual_F(PHpointer->GetOffset_w(), R, c);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntLoadResidual_F(Lpointer->GetOffset_w(), R, c);
    }
    this->contact_container->IntLoadResidual_F(contact_container->GetOffset_w(), R, c); //  does nothing, but just in case implemented in future.
}

/// Increment a vector R with a term that has M multiplied a given vector w:
///    R += c*M*w
void ChSystem::LoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                               const ChVectorDynamic<>& w,  ///< the w vector
                               const double c               ///< a scaling factor
                               ) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntLoadResidual_Mv(Bpointer->GetOffset_w(), R, w, c);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntLoadResidual_Mv(PHpointer->GetOffset_w(), R, w, c);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntLoadResidual_Mv(Lpointer->GetOffset_w(), R, w, c);
    }
    this->contact_container->IntLoadResidual_Mv(contact_container->GetOffset_w(), R, w, c); //  does nothing, but just in case implemented in future.
}

/// Increment a vectorR with the term Cq'*L:
///    R += c*Cq'*L
void ChSystem::LoadResidual_CqL(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                const ChVectorDynamic<>& L,  ///< the L vector
                                const double c               ///< a scaling factor
                                ) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntLoadResidual_CqL(Bpointer->GetOffset_L(), R, L, c);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntLoadResidual_CqL(PHpointer->GetOffset_L(), R, L, c);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntLoadResidual_CqL(Lpointer->GetOffset_L(), R, L, c);
    }
    this->contact_container->IntLoadResidual_CqL(contact_container->GetOffset_L(), R, L, c);
}

/// Increment a vector Qc with the term C:
///    Qc += c*C
void ChSystem::LoadConstraint_C(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*C
                                const double c,         ///< a scaling factor
                                const bool mdo_clamp,   ///< enable optional clamping of Qc
                                const double mclam      ///< clamping value
                                ) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntLoadConstraint_C(Bpointer->GetOffset_L(), Qc, c, mdo_clamp, mclam);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntLoadConstraint_C(PHpointer->GetOffset_L(), Qc, c, mdo_clamp, mclam);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntLoadConstraint_C(Lpointer->GetOffset_L(), Qc, c, mdo_clamp, mclam);
    }
    this->contact_container->IntLoadConstraint_C(contact_container->GetOffset_L(), Qc, c, mdo_clamp, mclam);
}

/// Increment a vector Qc with the term Ct = partial derivative dC/dt:
///    Qc += c*Ct
void ChSystem::LoadConstraint_Ct(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*Ct
                                 const double c          ///< a scaling factor
                                 ) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        if (Bpointer->IsActive())
            Bpointer->IntLoadConstraint_Ct(Bpointer->GetOffset_L(), Qc, c);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->IntLoadConstraint_Ct(PHpointer->GetOffset_L(), Qc, c);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        if (Lpointer->IsActive())
            Lpointer->IntLoadConstraint_Ct(Lpointer->GetOffset_L(), Qc, c);
    }
    this->contact_container->IntLoadConstraint_Ct(contact_container->GetOffset_L(), Qc, c); //  does nothing, but just in case implemented in future.
}

//////////////////////////////////
////////
////////    COLLISION OPERATIONS
////////

int ChSystem::GetNcontacts() {
    return this->contact_container->GetNcontacts();
}

void ChSystem::SynchronizeLastCollPositions() {
#pragma omp parallel for
    for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];

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

// Update all positions of collision models
#pragma omp parallel for
    for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        Bpointer->SyncCollisionModels();
    }
#pragma omp parallel for
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        PHpointer->SyncCollisionModels();
    }

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

    collision_system->ReportContacts(this->contact_container);

    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        if (ChContactContainerBase* mcontactcontainer = dynamic_cast<ChContactContainerBase*>(PHpointer)) {
            collision_system->ReportContacts(mcontactcontainer);
        }
        if (ChProximityContainerBase* mproximitycontainer = dynamic_cast<ChProximityContainerBase*>(PHpointer)) {
            collision_system->ReportProximities(mproximitycontainer);
        }
    }

    // If some other collision engine could add further ChLinkContact into the list..
    if (collision_callback)
        collision_callback->PerformCustomCollision(this);

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

    // Compute contacts and create contact constraints
    ComputeCollisions();

    // Counts dofs, statistics, etc.
    Setup();

    // Update everything - and put to sleep bodies that need it.
    // Note that we do not update visualization assets at this point.
    Update(false);

    // Re-wake the bodies that cannot sleep because they are in contact with
    // some body that is not in sleep state.
    WakeUpSleepingBodies();

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

#pragma omp parallel for
    for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];

        // EULERO INTEGRATION: pos+=v_new*dt  (do not do this, if GPU already computed it)
        Bpointer->VariablesQbIncrementPosition(step);
        // Set body speed, and approximates the acceleration by differentiation.
        Bpointer->VariablesQbSetSpeed(step);

        // Now also updates all markers & forces
        Bpointer->Update(this->ChTime);
    }

#pragma omp parallel for
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];

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

    // Compute contacts and create contact constraints
    ComputeCollisions();

    // Counts dofs, statistics, etc.
    Setup();

    // Update everything - and put to sleep bodies that need it.
    // Note that we do not update visualization assets at this point.
    Update(false);

    // Re-wake the bodies that cannot sleep because they are in contact with
    // some body that is not in sleep state.
    WakeUpSleepingBodies();

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

#pragma omp parallel for
    for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];

        // EULERO INTEGRATION: pos+=v_new*dt
        Bpointer->VariablesQbIncrementPosition(step);
        // Set body speed, and approximates the acceleration by differentiation.
        Bpointer->VariablesQbSetSpeed(step);

        // Now also updates all markers & forces
        // Bpointer->UpdateALL(this->ChTime); // not needed - will be done later anyway
    }

#pragma omp parallel for
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];

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
#pragma omp parallel for
        for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
        {
            ChBody* Bpointer = bodylist[ip];

            Bpointer->VariablesQbIncrementPosition(1.0);  // pos+=Dpos
            // Now also updates all markers & forces
            Bpointer->Update(this->ChTime);
        }

#pragma omp parallel for
        for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
        {
            ChPhysicsItem* PHpointer = otherphysicslist[ip];

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

    // Compute contacts and create contact constraints
    ComputeCollisions();

    // Counts dofs, statistics, etc. (not needed because already in Advance()...? )
    Setup();

    // Update everything - and put to sleep bodies that need it (not needed because already in Advance()...? )
    // No need to update visualization assets here.
    Update(false);

    // Re-wake the bodies that cannot sleep because they are in contact with
    // some body that is not in sleep state.
    WakeUpSleepingBodies();

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

#pragma omp parallel for
            for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
            {
                ChBody* Bpointer = bodylist[ip];
                Bpointer->VariablesQbIncrementPosition(1.0);  // pos += Dpos
                Bpointer->Update(this->ChTime);
            }

#pragma omp parallel for
            for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
            {
                ChPhysicsItem* PHpointer = otherphysicslist[ip];
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
#pragma omp parallel for
        for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
        {
            ChBody* Bpointer = bodylist[ip];
            Bpointer->VariablesQbSetSpeed(step);
            Bpointer->Update(this->ChTime);
        }

#pragma omp parallel for
        for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
        {
            ChPhysicsItem* PHpointer = otherphysicslist[ip];
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
    Setup();
    Update();

    int old_maxsteps = this->GetIterLCPmaxItersSpeed();
    this->SetIterLCPmaxItersSpeed(300);

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
    Setup();
    Update();

    int old_maxsteps = this->GetIterLCPmaxItersSpeed();
    this->SetIterLCPmaxItersSpeed(300);

    ChStaticNonLinearAnalysis manalysis(*this);
    manalysis.SetMaxiters(nsteps);

    // Perform analysis
    manalysis.StaticAnalysis();

    this->SetIterLCPmaxItersSpeed(old_maxsteps);

    return 0;
}

// **** PERFORM THE STATIC ANALYSIS, FINDING THE STATIC
// **** EQUILIBRIUM OF THE SYSTEM, WITH ITERATIVE SOLUTION

int ChSystem::DoStaticRelaxing() {
    int err = 0;
    int reached_tolerance = FALSE;


    if (ncoords > 0) {
        if (ndof >= 0) {
            for (int m_iter = 0; m_iter < STATIC_MAX_STEPS; m_iter++) {
#pragma omp parallel for
                for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
                {
                    ChBody* Bpointer = bodylist[ip];
                    // Set no body speed and no body accel.
                    Bpointer->SetNoSpeedNoAcceleration();
                }
#pragma omp parallel for
                for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
                {
                    ChPhysicsItem* PHpointer = otherphysicslist[ip];
                    PHpointer->SetNoSpeedNoAcceleration();
                }

                double m_undotime = this->GetChTime();
                DoFrameDynamics(m_undotime +
                                (step * 1.8) * (((double)STATIC_MAX_STEPS - (double)m_iter)) /
                                    (double)STATIC_MAX_STEPS);
                this->SetChTime(m_undotime);
            }

#pragma omp parallel for
            for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
            {
                ChBody* Bpointer = bodylist[ip];
                // Set no body speed and no body accel.
                Bpointer->SetNoSpeedNoAcceleration();
            }

#pragma omp parallel for
            for (int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
            {
                ChPhysicsItem* PHpointer = otherphysicslist[ip];
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

int ChSystem::DoStepDynamics(double m_step) {
    this->step = m_step;
    return Integrate_Y();
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

        if (left_time < 0.0000000000001)
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
    // clearances in costraints.
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
    ChObj::ArchiveOUT(marchive);

    // serialize all member data:
    
    marchive << CHNVP(G_acc);

    //marchive << CHNVP(bodylist);
    // do rather a custom array save:
    marchive.out_array_pre("bodies", bodylist.size(), "ChBody");
    for (int i = 0; i < bodylist.size(); i++) {
        bodylist[i]->AddRef(); // hack: since in list are not as shared pointers
        ChSharedPtr<ChBody> a_body(bodylist[i]); // wrap into shared ptr
        marchive << CHNVP(a_body,"");
        marchive.out_array_between(bodylist.size(), "bodies");
    }
    marchive.out_array_end(bodylist.size(), "bodies");

    //marchive << CHNVP(linklist);
    // do rather a custom array save:
    marchive.out_array_pre("links", linklist.size(), "ChLink");
    for (int i = 0; i < linklist.size(); i++) {
        linklist[i]->AddRef(); // hack: since in list are not as shared pointers
        ChSharedPtr<ChLink> a_link(linklist[i]); // wrap into shared ptr
        marchive << CHNVP(a_link,"");
        marchive.out_array_between(linklist.size(), "links");
    }
    marchive.out_array_end(linklist.size(), "links");

    //***TODO*** complete...
}

/// Method to allow de serialization of transient data from archives.
void ChSystem::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChObj::ArchiveIN(marchive);

    // stream in all member data:
    marchive >> CHNVP(G_acc);

    //marchive >> CHNVP(bodylist);
    // do rather a custom array load:
    this->RemoveAllBodies();
    size_t num_bodies;
    marchive.in_array_pre("bodies", num_bodies);
    for (int i = 0; i < num_bodies; i++) {
        ChSharedPtr<ChBody> a_body;
        marchive >> CHNVP(a_body,"");
        this->AddBody(a_body);
        marchive.in_array_between("bodies");
    }
    marchive.in_array_end("bodies");

    //marchive >> CHNVP(linklist);
    // do rather a custom array load:
    this->RemoveAllLinks();
    size_t num_links;
    marchive.in_array_pre("links", num_links);
    for (int i = 0; i < num_links; i++) {
        ChSharedPtr<ChLink> a_link;
        marchive >> CHNVP(a_link,"");
        this->AddLink(a_link);
        marchive.in_array_between("links");
    }
    marchive.in_array_end("links");

    //***TODO*** complete...
}


void ChSystem::StreamOUT(ChStreamOutBinary& mstream) {
    // class version number
    mstream.VersionWrite(7);

    // serialize parent class too
    ChObj::StreamOUT(mstream);

    // stream out all member data
    mstream << GetEndTime();
    mstream << GetStep();
    mstream << GetStepMin();
    mstream << GetStepMax();
    mstream << GetTol();
    mstream << GetNormType();
    mstream << GetMaxiter();
    mstream << (int)GetIntegrationType();
    mstream << (int)0;     // v7
    mstream << (int)0;     // v7
    mstream << (int)0;     // v7
    mstream << (int)0;     // GetBaumgarteStabilize();
    mstream << (int)0;     // v7
    mstream << (double)0;  // v7
    mstream << (int)0;     // v7
    mstream << (int)0;     // v7
    mstream << (double)0;  // v7
    mstream << G_acc;
    mstream << (int)0;
    mstream << (int)0;     // v7
    mstream << (int)0;     // v7
    mstream << (double)0;  // v7
    mstream << (double)0;  // v7
    mstream << (int)0;     // v7
    mstream << GetScriptForStartFile();
    mstream << GetScriptForUpdateFile();
    mstream << GetScriptForStepFile();
    mstream << GetScriptFor3DStepFile();
    mstream << (int)0;  // v7
    mstream << GetMinBounceSpeed();
    // v2
    mstream << iterLCPmaxIters;
    mstream << iterLCPmaxItersStab;
    mstream << (int)0;
    mstream << (int)GetLcpSolverType();
    // v3,v4
    mstream << (int)0;  // GetFrictionProjection();
    // v5
    mstream << parallel_thread_number;
    mstream << max_penetration_recovery_speed;
    // v6
    mstream << use_sleeping;
}

void ChSystem::StreamIN(ChStreamInBinary& mstream) {
    // class version number
    int version = mstream.VersionRead();

    // deserialize parent class too
    ChObj::StreamIN(mstream);

    // stream in all member data
    double mdouble;
    int mint;
    Vector mvector;
    char buffer[250];
    mstream >> mdouble;
    SetEndTime(mdouble);
    mstream >> mdouble;
    SetStep(mdouble);
    mstream >> mdouble;
    SetStepMin(mdouble);
    mstream >> mdouble;
    SetStepMax(mdouble);
    mstream >> mdouble;
    SetTol(mdouble);
    mstream >> mint;
    SetNormType(mint);
    mstream >> mint;
    SetMaxiter(mint);
    mstream >> mint;
    SetIntegrationType((eCh_integrationType)mint);
    mstream >> mint;     // SetOrder(mint);
    mstream >> mint;     // SetMultisteps(mint);
    mstream >> mint;     // SetAdaption (mint);
    mstream >> mint;     // SetBaumgarteStabilize(mint);
    mstream >> mint;     // SetDynaclose(mint);
    mstream >> mdouble;  // SetDynatol(mdouble);
    mstream >> mint;     // SetPredict(mint);
    mstream >> mint;     // SetPredorder(mint);
    mstream >> mdouble;  // SetStifftol(mdouble);
    mstream >> mvector;
    Set_G_acc(mvector);
    mstream >> mint;     // SetXYmode(mint);
    mstream >> mint;     // SetNsClosePos(mint);
    mstream >> mint;     // SetNsCloseSpeed(mint);
    mstream >> mdouble;  // SetMonolattol(mdouble);
    mstream >> mdouble;  // SetIntegrtol(mdouble);
    mstream >> mint;     // SetAutoAssembly(mint);
    mstream >> buffer;
    SetScriptForStartFile(buffer);
    mstream >> buffer;
    SetScriptForUpdateFile(buffer);
    mstream >> buffer;
    SetScriptForStepFile(buffer);
    mstream >> buffer;
    SetScriptFor3DStepFile(buffer);
    mstream >> mint;  // SetMaxStepsCollide(mint);
    mstream >> mdouble;
    SetMinBounceSpeed(mdouble);

    if (version >= 2) {
        mstream >> iterLCPmaxIters;
        mstream >> iterLCPmaxItersStab;
        mstream >> mint;
        mstream >> mint;
        SetLcpSolverType((eCh_lcpSolver)mint);
    }
    if (version >= 3) {
        mstream >> mint;  // SetFrictionProjection((eCh_frictionProjection) mint);
        if (version == 3) {
        };  // SetFrictionProjection(FRI_CONEORTHO); // for v3, ortho proj anyway
    }
    if (version >= 5) {
        mstream >> parallel_thread_number;
        mstream >> max_penetration_recovery_speed;
    }
    if (version >= 6) {
        mstream >> use_sleeping;
    }
}

void ChSystem::StreamOUT(ChStreamOutAscii& mstream) {
    // serialize parent class too
    ChObj::StreamOUT(mstream);

    // stream out all member data
    mstream << "tEnd      : " << GetEndTime() << "\nstepSize   : " << GetStep() << "\nstepMin    : " << GetStepMin()
            << "\nstepMax    : " << GetStepMax() << "\ntol        : " << GetTol() << "\nnormType   : " << GetNormType()
            << "\nMaxIter    : " << GetMaxiter() << "\nIntegration: " << (int)GetIntegrationType()

            << "\ngravity  : " << G_acc

            << "\nminBounceSpeed   : " << GetMinBounceSpeed()
            // v2
            << "\nLCPmaxIter       : " << iterLCPmaxIters << "\nLCPmaxIterStab   :  " << iterLCPmaxItersStab
            << "\nmaxRecovSpeed    : " << max_penetration_recovery_speed
            << "\nLCPSolverType    : " << (int)GetLcpSolverType();
}

#define CH_CHUNK_END 1234
#define CH_CHUNK_START 4321

int ChSystem::StreamINall(ChStreamInBinary& m_file) {
    int mchunk = 0;
    ChBody* newbody = NULL;
    ChLink* newlink = NULL;
    ChPhysicsItem* newitem = NULL;

    // class version number
    int version = m_file.VersionRead();

    // 0) reset system to have no sub object child
    this->Clear();

    // 1) read system class data...
    m_file >> *this;

    // 2a) read how many bodies
    int mnbodies = 0;
    m_file >> mnbodies;

    // 2b) read  bodies
    for (int i = 0; i < mnbodies; i++) {
        ChSharedPtr<ChBody> newbody(new ChBody);
        this->AddBody(newbody);

        if (!newbody->StreamINall(m_file))
            throw ChException("Cannot read body data");
    }

    // 3a) read how many links
    int mnlinks = 0;
    m_file >> mnlinks;

    // 3b) read  links
    for (int j = 0; j < mnlinks; j++) {
        // read the link, using a special downcasting function Link_BinRead_Create which creates the
        // proper inherited object, depending on its class inheritance from base Link*

        m_file.AbstractReadCreate(&newlink);
        if (!newlink)
            throw ChException("Cannot read link data");

        ChSharedPtr<ChLink> shlink(newlink);
        this->AddLink(shlink);
    }

    // 3c) Rebuild link pointers to markers
    this->Reference_LM_byID();

    // 4a) read how many other physics item
    int mnitems = 0;
    m_file >> mnitems;

    // 4b) read physics items
    for (int j = 0; j < mnitems; j++) {
        // read the item, using a special downcasting function which creates the
        // proper inherited object, depending on its class inheritance from base ChPhysicsItem*

        m_file.AbstractReadCreate(&newitem);
        if (!newitem)
            throw ChException("Cannot read ChPhysicsItem data.");

        ChSharedPtr<ChPhysicsItem> shitem(newitem);
        this->AddOtherPhysicsItem(shitem);
    }

    this->Setup();

    return 1;
}

int ChSystem::StreamOUTall(ChStreamOutBinary& m_file) {
    // class version number
    m_file.VersionWrite(2);

    // 1) write system class data...
    m_file << *this;

    // 2a) write how many bodies
    m_file << (int)bodylist.size();  // this->ListCount((ChObj**)&bodylist); ***SHAREDBODY***

    // 2b) write  bodies
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChBody* Bpointer = bodylist[ip];
        // write the body + child markers + forces
        if (!Bpointer->StreamOUTall(m_file))
            return 0;
    }

    // 3a) write how many links
    m_file << (int)linklist.size();

    // 3b) write links links
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];
        // Writethe link, using a special downcasting function Link_BinSave which saves also the
        // inheritance info, depending on link class inheritance from base Link*
        m_file.AbstractWrite(Lpointer);
    }

    // 4a) write how many other physics items
    m_file << (int)otherphysicslist.size();

    // 4b) write other physics item links
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];
        // Write the item, using a special downcasting function which saves also the
        // inheritance info, depending on class inheritance from base ChPhysicsItem*
        m_file.AbstractWrite(PHpointer);
    }

    m_file << (int)CH_CHUNK_END;

    return 1;
}

void ChSystem::ShowHierarchy(ChStreamOutAscii& m_file) {
    m_file << "\n   List of the " << (int)Get_bodylist()->size() << " added rigid bodies: \n";

    std::vector<ChBody*>::iterator ibody = Get_bodylist()->begin();
    while (ibody != Get_bodylist()->end()) {
        m_file << "     BODY:       " << (*ibody)->GetName() << "\n";

        std::vector<ChMarker*>::const_iterator imarker = (*ibody)->GetMarkerList().begin();
        while (imarker != (*ibody)->GetMarkerList().end()) {
            m_file << "        MARKER:   " << (*imarker)->GetName() << "\n";
            imarker++;
        }

        std::vector<ChForce*>::const_iterator iforce = (*ibody)->GetForceList().begin();
        while (iforce != (*ibody)->GetForceList().end()) {
            m_file << "        FORCE:   " << (*iforce)->GetName() << "\n";
            iforce++;
        }

        ibody++;
    }

    m_file << "\n   List of the " << (int)Get_linklist()->size() << " added links: \n";

    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChLink* Lpointer = linklist[ip];

        m_file << "     LINK:       " << Lpointer->GetName() << "\n";
        if (ChLinkMarkers* malink = ChDynamicCast(ChLinkMarkers, Lpointer)) {
            if (malink->GetMarker1())
                m_file << "        marker1:     " << malink->GetMarker1()->GetName() << "\n";
            if (malink->GetMarker2())
                m_file << "        marker2:     " << malink->GetMarker2()->GetName() << "\n";
        }
    }

    m_file << "\n   List of other " << (int)otherphysicslist.size() << " added physic items: \n";

    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChPhysicsItem* PHpointer = otherphysicslist[ip];

        m_file << "     PHYSIC ITEM :       " << PHpointer->GetName() << "\n";
    }

    m_file << "\n\nFlat ChPhysicalItem list (class name - object name):----- \n\n";

    IteratorAllPhysics mphiter(this);
    while (mphiter.HasItem()) {
        m_file << "  " << mphiter->GetRTTI()->GetName() << "  -  " << mphiter->GetName() << "\n";
        ++mphiter;
    }
    m_file << "\n\n";
}

int ChSystem::FileProcessChR(ChStreamInBinary& m_file) {
    int mchunk = 0;

    m_file >> mchunk;
    if (mchunk != CH_CHUNK_START)
        throw ChException("Not a ChR data file.");

    this->StreamINall(m_file);

    m_file >> mchunk;
    if (mchunk != CH_CHUNK_END)
        throw ChException("The end of ChR data file is badly formatted.");

    return 1;
}

int ChSystem::FileWriteChR(ChStreamOutBinary& m_file) {
    m_file << (int)CH_CHUNK_START;

    this->StreamOUTall(m_file);

    m_file << (int)CH_CHUNK_END;

    return 1;
}

// process a scripting instruction file
int ChSystem::FileProcessJS(char* m_file) {
    if (!this->scriptEngine)
        return 0;

    ChScript* mscript = this->scriptEngine->CreateScript();
    if (!this->scriptEngine->ExecuteScript(*mscript))
        return 0;
    delete mscript;
    return 1;
}

//////////////////////////////////////////////////////////////////

ChSystem::IteratorBodies& ChSystem::IteratorBodies::operator=(const IteratorBodies& other) {
    node_ = other.node_;
    return (*this);
}
bool ChSystem::IteratorBodies::operator==(const IteratorBodies& other) {
    return (node_ == other.node_);
}
bool ChSystem::IteratorBodies::operator!=(const IteratorBodies& other) {
    return (node_ != other.node_);
}
ChSystem::IteratorBodies& ChSystem::IteratorBodies::operator++() {
    node_++;
    return (*this);
}
ChSharedPtr<ChBody> ChSystem::IteratorBodies::operator*() {
    (*node_)->AddRef();  // needed because ...
    return (ChSharedPtr<ChBody>(
        (*node_)));  // .. here I am not getting a new() data, but a reference to something created elsewhere
}
ChSystem::IteratorBodies ChSystem::IterBeginBodies() {
    return (IteratorBodies(this->bodylist.begin()));
}
ChSystem::IteratorBodies ChSystem::IterEndBodies() {
    return (IteratorBodies(this->bodylist.end()));
}

//////////////////////////////////////////////////////////////////

ChSystem::IteratorLinks& ChSystem::IteratorLinks::operator=(const IteratorLinks& other) {
    node_ = other.node_;
    return (*this);
}
bool ChSystem::IteratorLinks::operator==(const IteratorLinks& other) {
    return (node_ == other.node_);
}
bool ChSystem::IteratorLinks::operator!=(const IteratorLinks& other) {
    return (node_ != other.node_);
}
ChSystem::IteratorLinks& ChSystem::IteratorLinks::operator++() {
    node_++;
    return (*this);
}
ChSharedPtr<ChLink> ChSystem::IteratorLinks::operator*() {
    (*node_)->AddRef();  // needed because ...
    return (ChSharedPtr<ChLink>(
        (*node_)));  // .. here I am not getting a new() data, but a reference to something created elsewhere
}
ChSystem::IteratorLinks ChSystem::IterBeginLinks() {
    return (IteratorLinks(this->linklist.begin()));
}
ChSystem::IteratorLinks ChSystem::IterEndLinks() {
    return (IteratorLinks(this->linklist.end()));
}

//////////////////////////////////////////////////////////////////

ChSystem::IteratorOtherPhysicsItems& ChSystem::IteratorOtherPhysicsItems::operator=(
    const IteratorOtherPhysicsItems& other) {
    node_ = other.node_;
    return (*this);
}
bool ChSystem::IteratorOtherPhysicsItems::operator==(const IteratorOtherPhysicsItems& other) {
    return (node_ == other.node_);
}
bool ChSystem::IteratorOtherPhysicsItems::operator!=(const IteratorOtherPhysicsItems& other) {
    return (node_ != other.node_);
}
ChSystem::IteratorOtherPhysicsItems& ChSystem::IteratorOtherPhysicsItems::operator++() {
    node_++;
    return (*this);
}
ChSharedPtr<ChPhysicsItem> ChSystem::IteratorOtherPhysicsItems::operator*() {
    (*node_)->AddRef();  // needed because ...
    return (ChSharedPtr<ChPhysicsItem>(
        (*node_)));  // .. here I am not getting a new() data, but a reference to something created elsewhere
}
ChSystem::IteratorOtherPhysicsItems ChSystem::IterBeginOtherPhysicsItems() {
    return (IteratorOtherPhysicsItems(this->otherphysicslist.begin()));
}
ChSystem::IteratorOtherPhysicsItems ChSystem::IterEndOtherPhysicsItems() {
    return (IteratorOtherPhysicsItems(this->otherphysicslist.end()));
}

//////////////////////////////////////////////////////////////////

ChSystem::IteratorPhysicsItems::IteratorPhysicsItems(ChSystem* msys) {
    this->msystem = msys;
    // RewindToBegin();
    node_body = msystem->Get_bodylist()->begin();
    node_link = msystem->Get_linklist()->begin();
    node_otherphysics = msystem->Get_otherphysicslist()->begin();
    stage = 0;
    mptr = 0;
    this->operator++();  // initialize with 1st available item
}
ChSystem::IteratorPhysicsItems::IteratorPhysicsItems() {
    this->msystem = 0;
    this->mptr = 0;
    this->stage = 9999;
}

ChSystem::IteratorPhysicsItems::~IteratorPhysicsItems() {
}
/*
void ChSystem::IteratorPhysicsItems::RewindToBegin()
{
  node_body   = msystem->Get_bodylist()->begin();
  node_link	  = msystem->Get_linklist()->begin();
  node_otherphysics	 = msystem->Get_otherphysicslist()->begin();
  stage = 0;
  mptr = 0;
  this->operator++(); // initialize with 1st available item
}

bool ChSystem::IteratorPhysicsItems::ReachedEnd()
{
  if (stage == 9999)
      return true;
  return false;
}
*/
bool ChSystem::IteratorPhysicsItems::HasItem() {
    if (mptr)
        return true;
    return false;
}

ChSystem::IteratorPhysicsItems& ChSystem::IteratorPhysicsItems::operator=(const ChSystem::IteratorPhysicsItems& other) {
    msystem = other.msystem;
    node_body = other.node_body;
    node_link = other.node_link;
    node_otherphysics = other.node_otherphysics;
    stage = other.stage;
    mptr = other.mptr;
    return (*this);
}

bool ChSystem::IteratorPhysicsItems::operator==(const ChSystem::IteratorPhysicsItems& other) {
    return ((mptr == other.mptr) && (stage == other.stage) && (msystem == other.msystem));  //...***TO CHECK***
}

bool ChSystem::IteratorPhysicsItems::operator!=(const ChSystem::IteratorPhysicsItems& other) {
    return !(this->operator==(other));  //...***TO CHECK***
}

ChSystem::IteratorPhysicsItems& ChSystem::IteratorPhysicsItems::operator++() {
    switch (stage) {
        case 1: {
            node_body++;  // try next body
            if (node_body != msystem->Get_bodylist()->end()) {
                mptr = (*node_body);
                return (*this);
            }
            break;
        }
        case 2: {
            node_link++;  // try next link
            if (node_link != msystem->Get_linklist()->end()) {
                mptr = (*node_link);
                return (*this);
            }
            break;
        }
        case 3: {
            node_otherphysics++;  // try next otherphysics
            if (node_otherphysics != msystem->Get_otherphysicslist()->end()) {
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
                if (node_body != msystem->Get_bodylist()->end()) {
                    mptr = (*node_body);
                    return (*this);
                }
                break;
            }
            case 1: {
                stage = 2;
                if (node_link != msystem->Get_linklist()->end()) {
                    mptr = (*node_link);
                    return (*this);
                }
                break;
            }
            case 2: {
                stage = 3;
                if (node_otherphysics != msystem->Get_otherphysicslist()->end()) {
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
                mptr = 0;
                return (*this);
            }
        }  // end cases
    } while (true);

    return (*this);
}

ChSharedPtr<ChPhysicsItem> ChSystem::IteratorPhysicsItems::operator*() {
    mptr->AddRef();  // needed because ...
    return (ChSharedPtr<ChPhysicsItem>(
        mptr));  // .. here I am not getting a new() data, but a reference to something created elsewhere
}
ChSystem::IteratorPhysicsItems ChSystem::IterBeginPhysicsItems() {
    return (IteratorPhysicsItems(this));
}
ChSystem::IteratorPhysicsItems ChSystem::IterEndPhysicsItems() {
    return (IteratorPhysicsItems());
}

}  // END_OF_NAMESPACE____

/////////////////////////////////////// eof
