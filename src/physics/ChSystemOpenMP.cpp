//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChSystemOpenMP.cpp
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

#include "physics/ChSystemOpenMP.h"
#include "physics/ChGlobal.h"
//#include "physics/ChCollide.h"
#include "physics/ChBodyAuxRef.h"
#include "physics/ChContactContainer.h"
#include "physics/ChProximityContainerBase.h"

#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpSimplexSolver.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativeSymmSOR.h"
#include "lcp/ChLcpIterativeSORmultithread.h"
#include "lcp/ChLcpIterativeJacobi.h"
//#include "lcp/ChLcpIterativeMINRES.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "lcp/ChLcpIterativeBB.h"
#include "lcp/ChLcpIterativePCG.h"
#include "lcp/ChLcpSolverDEM.h"

#include "core/ChTimer.h"
#include "collision/ChCCollisionSystemBullet.h"
#include "collision/ChCModelBulletBody.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.
using namespace chrono::collision;

namespace chrono {

// Hierarchy-handling functions

#define Bpointer		    (*ibody)
#define HIER_BODY_INIT      std::vector<ChBody*>::iterator ibody = bodylist.begin();
#define HIER_BODY_NOSTOP    (ibody != bodylist.end())
#define HIER_BODY_NEXT	    ibody++;

#define Lpointer		    (*iterlink)
#define HIER_LINK_INIT      std::list<ChLink*>::iterator iterlink = linklist.begin();
#define HIER_LINK_NOSTOP    (iterlink != linklist.end())
#define HIER_LINK_NEXT	    iterlink++;

#define PHpointer		    (*iterotherphysics)
#define HIER_OTHERPHYSICS_INIT    std::list<ChPhysicsItem*>::iterator iterotherphysics = otherphysicslist.begin();
#define HIER_OTHERPHYSICS_NOSTOP  (iterotherphysics != otherphysicslist.end())
#define HIER_OTHERPHYSICS_NEXT	  iterotherphysics++;

#define Ppointer		    (*iterprobe)
#define HIER_PROBE_INIT      std::vector<ChProbe*>::iterator iterprobe = probelist.begin();
#define HIER_PROBE_NOSTOP    (iterprobe != probelist.end())
#define HIER_PROBE_NEXT	    iterprobe++;

#define Cpointer		    (*itercontrol)
#define HIER_CONTROLS_INIT      std::vector<ChControls*>::iterator itercontrol = controlslist.begin();
#define HIER_CONTROLS_NOSTOP    (itercontrol != controlslist.end())
#define HIER_CONTROLS_NEXT	    itercontrol++;

//////////////////////////////////////
//////////////////////////////////////
// CLASS FOR PHYSICAL SYSTEM

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChSystemOpenMP> a_registration_ChSystemOpenMP;

ChSystemOpenMP::ChSystemOpenMP(unsigned int max_objects, double scene_size, bool init_sys) :
        ChSystem() {
}

ChSystemOpenMP::~ChSystemOpenMP() {

}

void ChSystemOpenMP::Update()
{

    ChTimer<double>mtimer; mtimer.start(); // Timer for profiling


    //events->Record(CHEVENT_UPDATE); // Record an update event

                                    // Executes the "forUpdate" script, if any
    ExecuteScriptForUpdate();
                                    // Executes the "forUpdate" script
                                    // in all controls of controlslist
    ExecuteControlsForUpdate();

                                    // --------------------------------------
                                    // Spread state vector Y to bodies
                                    //    Y --> Bodies




    mtimer.stop();
    timer_update += mtimer();
}




void ChSystemOpenMP::LCPprepare(bool load_jacobians,
                               bool load_v,
                               double F_factor,
							   double K_factor,
							   double R_factor,
							   double M_factor,
                               double Ct_factor,
                               double C_factor,
                               double recovery_clamp,
                               bool do_clamp,
                               ChLcpSystemDescriptor& mdescriptor
                                )
{
    this->contact_container->Update(); // Update all contacts, if any
    this->contact_container->ConstraintsBiReset();
    mdescriptor.BeginInsertion(); // This resets the vectors of constr. and var. pointers.
    HIER_LINK_INIT
    while HIER_LINK_NOSTOP
    {
        Lpointer->Update(ChTime);
        Lpointer->ConstraintsBiReset();
        if (C_factor)
            Lpointer->ConstraintsBiLoad_C(C_factor, recovery_clamp, do_clamp);
        if (Ct_factor)
            Lpointer->ConstraintsBiLoad_Ct(Ct_factor);          // Ct
        if (F_factor)
            Lpointer->ConstraintsFbLoadForces(F_factor);        // f*dt
        if (load_jacobians)
            Lpointer->ConstraintsLoadJacobians();
        Lpointer->ConstraintsLiLoadSuggestedSpeedSolution();
        Lpointer->InjectConstraints(mdescriptor);
        HIER_LINK_NEXT
    }
#pragma omp parallel for
    for(int i=0; i<bodylist.size(); i++){
        bodylist[i]->Update(ChTime);
        if (this->GetUseSleeping()){ bodylist[i]->TrySleeping();}
         bodylist[i]->VariablesFbReset();
        if (F_factor)
             bodylist[i]->VariablesFbLoadForces(F_factor);          // f*dt
        if (load_v)
             bodylist[i]->VariablesQbLoadSpeed();                   // v_old

         bodylist[i]->InjectVariables(mdescriptor);
    }

    HIER_OTHERPHYSICS_INIT
    while HIER_OTHERPHYSICS_NOSTOP
    {
        PHpointer->Update(ChTime);
        PHpointer->VariablesFbReset();
               PHpointer->ConstraintsBiReset();
        if (F_factor)
            PHpointer->VariablesFbLoadForces(F_factor);         // f*dt
        if (load_v)
            PHpointer->VariablesQbLoadSpeed();                  // v_old
        if (C_factor)
            PHpointer->ConstraintsBiLoad_C(C_factor, recovery_clamp, do_clamp);
        if (Ct_factor)
            PHpointer->ConstraintsBiLoad_Ct(Ct_factor);         // Ct
        if (load_jacobians)
            PHpointer->ConstraintsLoadJacobians();
		if (K_factor || R_factor)
			PHpointer->KRMmatricesLoad(K_factor, R_factor, M_factor);
        PHpointer->ConstraintsLiLoadSuggestedSpeedSolution();
        PHpointer->InjectVariables(mdescriptor);
        PHpointer->InjectConstraints(mdescriptor);
		PHpointer->InjectKRMmatrices(mdescriptor);
        HIER_OTHERPHYSICS_NEXT
    }

    if (C_factor)
        contact_container->ConstraintsBiLoad_C(C_factor, recovery_clamp, do_clamp);
    if (F_factor)
        contact_container->ConstraintsFbLoadForces(F_factor);       // f*dt
    if (load_jacobians)
        contact_container->ConstraintsLoadJacobians();

    this->contact_container->ConstraintsLiLoadSuggestedSpeedSolution();
    this->contact_container->InjectConstraints(mdescriptor);
    mdescriptor.EndInsertion();
}

int ChSystemOpenMP::Integrate_Y_impulse_Anitescu() {
    int ret_code = TRUE;

        ChTimer<double> mtimer_step;
        mtimer_step.start();

        //events->Record(CHEVENT_TIMESTEP);

                                    // Executes the "forStep" script, if any
        ExecuteScriptForStep();
                                    // Executes the "forStep" script
                                    // in all controls of controlslist
        ExecuteControlsForStep();


        this->stepcount++;

        // Compute contacts and create contact constraints

        ComputeCollisions();


        Setup();    // Counts dofs, statistics, etc.


        Update();   // Update everything - and put to sleep bodies that need it

                    // Re-wake the bodies that cannot sleep because they are in contact with
                    // some body that is not in sleep state.
        //WakeUpSleepingBodies();


        ChTimer<double> mtimer_lcp;
        mtimer_lcp.start();


        //
        // Enforce velocity/impulses constraints ....................
        //

        // reset known-term vectors
        //LCPprepare_reset();

        // fill LCP known-term vectors with proper terms (forces, etc.):
        //
        // | M+dt^2*K+dt*R  -Cq'|*|v_new|- | [M]*v_old + f*dt      | = |0| ,  c>=0, l>=0, l*c=0;
        // | Cq              0  | |l    |  | -C/dt +min(-C/dt,vlim)|   |c|
        //

        LCPprepare(true,           // Cq,
                        true,           // add [M]*v_old to the known vector
                        GetStep(),      // f*dt
						GetStep()*GetStep(), // dt^2*K  (nb only non-Schur based solvers support K matrix blocks)
						GetStep(),		// dt*R   (nb only non-Schur based solvers support K matrix blocks)
						1.0,			// M (for FEM with non-lumped masses, add their mass-matrices)
                        1.0,            // Ct   (needed, for rheonomic motors)
                        1.0/GetStep(),  // C/dt
                        max_penetration_recovery_speed,  // vlim, max penetrations recovery speed (positive for exiting)
                        true, // do above max. clamping on -C/dt
                        *this->LCP_descriptor);

        // if warm start is used, can exploit cached multipliers from last step...
        //LCPprepare_Li_from_speed_cache();

        // make vectors of variables and constraints, used by the following LCP solver
        //LCPprepare_inject(*this->LCP_descriptor);


        // Solve the LCP problem.
        // Solution variables are new speeds 'v_new'
        GetLcpSolverSpeed()->Solve(
                                *this->LCP_descriptor
                                );          

        mtimer_lcp.stop();
        timer_lcp = mtimer_lcp();

        // stores computed multipliers in constraint caches, maybe useful for warm starting next step
        LCPresult_Li_into_speed_cache();

        // updates the reactions of the constraint
        LCPresult_Li_into_reactions(1.0/this->GetStep()) ; // R = l/dt  , approximately

        // perform an Eulero integration step (1st order stepping as pos+=v_new*dt)


        HIER_BODY_INIT
        while HIER_BODY_NOSTOP
        {
            // EULERO INTEGRATION: pos+=v_new*dt  (do not do this, if GPU already computed it)
            if (!use_GPU)
            {
                Bpointer->VariablesQbIncrementPosition(this->GetStep());
                // Set body speed, and approximates the acceleration by differentiation.
                Bpointer->VariablesQbSetSpeed(this->GetStep());
            }
            // Now also updates all markers & forces
            Bpointer->Update(this->ChTime);
            HIER_BODY_NEXT
        }
        HIER_OTHERPHYSICS_INIT
        while HIER_OTHERPHYSICS_NOSTOP
        {
            // EULERO INTEGRATION: pos+=v_new*dt  (do not do this, if GPU already computed it)
            if (!use_GPU)
            {
                PHpointer->VariablesQbIncrementPosition(this->GetStep());
                // Set body speed, and approximates the acceleration by differentiation.
                PHpointer->VariablesQbSetSpeed(this->GetStep());
            }
            // Now also updates all markers & forces
            PHpointer->Update(this->ChTime);
            HIER_OTHERPHYSICS_NEXT
        }

        this->ChTime = ChTime + GetStep();

        // Executes custom processing at the end of step
        CustomEndOfStep();

        // If there are some probe objects in the probe list,
        // tell them to record their variables (ususally x-y couples)
        RecordAllProbes();

        // Time elapsed for step..
        mtimer_step.stop();
        timer_step = mtimer_step();


        return (ret_code);
}
} // END_OF_NAMESPACE____

/////////////////////////////////////// eof

