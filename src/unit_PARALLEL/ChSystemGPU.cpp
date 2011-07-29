#include "ChSystemGPU.h"

namespace chrono {
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

int ChSystemGPU::Integrate_Y_impulse_Anitescu() {
	cudaEvent_t start, stop;
	float time;

	int ret_code = TRUE;

	ChTimer<double> mtimer_step;
	mtimer_step.start();

	this->stepcount++;

	// Compute contacts and create contact constraints

	ComputeCollisions();

	Setup(); // Counts dofs, statistics, etc.
	Update(); // Update everything - and put to sleep bodies that need it

	ChTimer<double> mtimer_lcp;
	mtimer_lcp.start();

	//
	// Enforce velocity/impulses constraints ....................
	//

	// reset known-term vectors
	LCPprepare_reset();

	// fill LCP known-term vectors with proper terms (forces, etc.):
	//
	// | M -Cq'|*|v_new|- | [M]*v_old + f*dt      | = |0| ,  c>=0, l>=0, l*c=0;
	// | Cq  0 | |l    |  | -C/dt +min(-C/dt,vlim)|   |c|
	//

	LCPprepare_load(true, // Cq,
			true, // v_old (needed for adding [M]*v_old to the known vector)
			GetStep(), // f*dt
			1.0, // Ct   (needed, for rheonomic motors)
			1.0 / GetStep(), // C/dt
			max_penetration_recovery_speed, // vlim, max penetrations recovery speed (positive for exiting)
			true); // do above max. clamping on -C/dt

	// if warm start is used, can exploit cached multipliers from last step...
	LCPprepare_Li_from_speed_cache();

	// make vectors of variables and constraints, used by the following LCP solver
	LCPprepare_inject(*this->LCP_descriptor);

	// Solve the LCP problem.
	// Solution variables are new speeds 'v_new'
	//START_TIMING(start,stop,time);
	GetLcpSolverSpeed()->Solve(*this->LCP_descriptor, true); // add [M]*v_old to the known vector
	//STOP_TIMING(start,stop,time);
	//printf("F: %f \n",time);
	mtimer_lcp.stop();
	timer_lcp = mtimer_lcp();
	// stores computed multipliers in constraint caches, maybe useful for warm starting next step
	LCPresult_Li_into_speed_cache();
	// updates the reactions of the constraint
	LCPresult_Li_into_reactions(1.0 / this->GetStep()); // R = l/dt  , approximately

	// perform an Eulero integration step (1st order stepping as pos+=v_new*dt)
	/*
	 HIER_BODY_INIT
	 while HIER_BODY_NOSTOP
	 {
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
	 CustomEndOfStep();*/
	this->ChTime = ChTime + GetStep();
	// Time elapsed for step..
	mtimer_step.stop();
	timer_step = mtimer_step();

	return (ret_code);

}

}
