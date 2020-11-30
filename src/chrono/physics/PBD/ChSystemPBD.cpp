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
// Authors: Simone Benatti
// =============================================================================
//
// Physical system in which contact is modeled using a non-smooth
// (complementarity-based) method.
//
// =============================================================================

#include <algorithm>

#include "chrono/physics/PBD/ChSystemPBD.h"
#include "chrono/physics/PBD/ChContactContainerPBD.h"
#include "chrono/physics/ChProximityContainer.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/collision/ChCollisionSystemBullet.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSystemPBD)

ChSystemPBD::ChSystemPBD(bool init_sys)
    : ChSystem() {
    if (init_sys) {
        // Set default contact container
        contact_container = chrono_types::make_shared<ChContactContainerPBD>();
        contact_container->SetSystem(this);

        // Set default collision engine
        collision_system = chrono_types::make_shared<collision::ChCollisionSystemBullet>();

        // Set the system descriptor
        descriptor = chrono_types::make_shared<ChSystemDescriptor>();

        // Set default solver
		// Not called by Integrate_Y. Users might call it through DoAssembly and such.
        SetSolverType(ChSolver::Type::PSOR);
    }

    // Set default collision envelope and margin.
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.03);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.01);
}

ChSystemPBD::ChSystemPBD(const ChSystemPBD& other) : ChSystem(other) {}

void ChSystemPBD::SetContactContainer(std::shared_ptr<ChContactContainer> container) {
    if (std::dynamic_pointer_cast<ChContactContainerPBD>(container))
        ChSystem::SetContactContainer(container);
}

// -----------------------------------------------------------------------------
//  PERFORM INTEGRATION STEP  using pluggable timestepper
// -----------------------------------------------------------------------------


bool ChSystemPBD::Integrate_Y() {
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
	//TODO: I had to change ChSystem::ManageSleepingBodies from private to protected. 
	// Not sure if this is useful, maybe skip smth is abody is sleeping?
	ManageSleepingBodies();
	// Prepare lists of variables and constraints.
	// TODO: check if this is needed by  PBD (constrint matrix is not, but maybe we process the state here)
	DescriptorPrepareInject(*descriptor);
	/// Qc matrix not used by PBD
	/*
	// No need to update counts and offsets, as already done by the above call (in ChSystemDescriptor::EndInsertion)
	////descriptor->UpdateCountsAndOffsets();
	// Set some settings in timestepper object
	timestepper->SetQcDoClamp(true);
	timestepper->SetQcClamping(max_penetration_recovery_speed);
	if (std::dynamic_pointer_cast<ChTimestepperHHT>(timestepper) ||
		std::dynamic_pointer_cast<ChTimestepperNewmark>(timestepper))
		timestepper->SetQcDoClamp(false);
		*/

		// If needed, update everything. No need to update visualization assets here.
	if (!PBD_isSetup) {
		PBDSetup();
		PBD_isSetup = true;
	}

	// PERFORM TIME STEP HERE!
	{
		CH_PROFILE("Advance");
		timer_advance.start();
		// TODO: change next line with PBD advance loop
		// TODO: links must be processed to fit into PBD forumulation. This cannot be done at each timestep but neither upon initialization.
		// We might add a "PBD_Setup" flag taht is called at most one.
		this->Advance();
		timer_advance.stop();
	}

	// Executes custom processing at the end of step
	CustomEndOfStep();

	// Call method to gather contact forces/torques in rigid bodies
	/// Not needed in PBD.
	//contact_container->ComputeContactForces();

	// Time elapsed for step
	timer_step.stop();

	// Tentatively mark system as unchanged (i.e., no updated necessary)
	is_updated = true;

	return true;
}


void ChSystemPBD::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSystemPBD>();

    // serialize parent class
    ChSystem::ArchiveOUT(marchive);

    // serialize all member data:
}

// Method to allow de serialization of transient data from archives.
void ChSystemPBD::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChSystemPBD>();

    // deserialize parent class
    ChSystem::ArchiveIN(marchive);

    // stream in all member data:
}

void ChSystemPBD::PBDSetup() {
	// create the list of links using PBD formulation
	for (auto& value : Get_linklist()) {
		if (dynamic_cast<const ChLinkLock*>(value.get()) != nullptr) {
			ChLinkLock* linkll = dynamic_cast< ChLinkLock*>(value.get());
			auto pbdlink = chrono_types::make_shared<ChLinkPBDLock>(linkll);
			linklistPBD.push_back(pbdlink);
		}
		else if (dynamic_cast<const ChLinkMateGeneric*>(value.get()) != nullptr) {
			ChLinkMateGeneric* linkmg = dynamic_cast<ChLinkMateGeneric*>(value.get());
			auto pbdlink = chrono_types::make_shared<ChLinkPBDMate>(linkmg);
			linklistPBD.push_back(pbdlink);
		}
		else
		{
			throw std::invalid_argument("One or more of the system links cannot be treated as PBD link");
		}
		
	}
	// The gyroscopic term is evaluated within the PBD loop
	for (auto& body : Get_bodylist()) {
		body->SetNoGyroTorque(true);
	}
	// allocate the proper amount of vectors and quaternions
	int n = Get_bodylist().size();
	x_prev.resize(n);
	//x.reserve(n);
	q_prev.resize(n);
	//q.reserve(n);
	//omega.reserve(n);
	//v.reserve(n);
}

void ChSystemPBD::SolvePositions() {
	for (auto& link : linklistPBD) {
		link->SolvePositions();
	}
}

// Implementation af algorithm 2 from Detailed Rigid Body Simulation with Extended Position Based Dynamics, Mueller et al.
void ChSystemPBD::Advance() {
	int n = Get_bodylist().size();
	double h = step / substeps;
	for (int i = 0; i < substeps; i++) {
		for (int j = 0; j < n; j++) {
			std::shared_ptr<ChBody> body = Get_bodylist()[j];
			if (body->GetBodyFixed() == true) {
				continue;
			}
			x_prev[j] = body->GetPos();
			q_prev[j] = body->GetRot();
			auto v = body->GetPos_dt() + h * (1 / body->GetMass()) * body->GetAppliedForce();
			body->SetPos_dt(v);
			body->SetPos(x_prev[j] + h * v);
			// gyroscopic effects also in body->ComputeGyro
			auto omega = body->GetWvel_loc() + body->GetInvInertia() * (body->GetAppliedTorque() - body->GetWvel_loc().Cross(body->GetInertia() *  body->GetWvel_loc()))*h;
			body->SetWvel_loc(omega);
			// !!! different multiplication order within Qdt_from_Wrel w.r.t. the paper
			// due to local vs global omegas and different  quaternion definition !!!
			ChQuaternion<double> dqdt;
			// dq/dt =  1/2 {q}*{0,w_rel}
			dqdt.Qdt_from_Wrel(omega[j], q_prev[j]);
			// q1 = q0 + dq/dt * h
			body->SetRot((q_prev[j] + dqdt *h).Normalize());
		}
		// Correct positions to respect constraints. "numPosIters"set to 1 according to the paper
		SolvePositions();
		// Update velocities to take corrected positions into account
		for (int j = 0; j < n; j++) {
			std::shared_ptr<ChBody> body = Get_bodylist()[j];
			if (body->GetBodyFixed()) {
				continue;
			}
			body->SetPos_dt((body->GetPos() - x_prev[j]) / h);
			// q_old^-1 * q instead of q * q_old^-1 for the same reason
			ChQuaternion<double> deltaq = q_prev[j].GetInverse() *body->GetRot();
			ChVector<double> omega_us = deltaq.GetVector()*(2 / h);
			body->SetWvel_loc((deltaq.e0() >= 0) ? omega_us : -omega_us);
		}
		// Scatter updated state

		// SolveVelocities();

		T += h;
		
	}
	// Scatter Gather influences only collision detaction -> do after substepping
	StateSetup(sys_state, sys_state_delta, sys_acc);
	StateGather(sys_state, sys_state_delta, T);
	StateScatter(sys_state, sys_state_delta, T, true);
}
}  // end namespace chrono
