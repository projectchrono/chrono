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
//   ChSystemDEM.cpp
//
///////////////////////////////////////////////////

#include "physics/ChSystemDEM.h"
#include "physics/ChContactContainerDEM.h"

#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpSolverDEM.h"

#include "collision/ChCCollisionSystemBullet.h"


namespace chrono
{


ChClassRegister<ChSystemDEM> a_registration_ChSystemDEM;


ChSystemDEM::ChSystemDEM(unsigned int max_objects, double scene_size)
:	ChSystem(max_objects, scene_size, false)
{
	LCP_descriptor = new ChLcpSystemDescriptor;
	LCP_descriptor->SetNumThreads(parallel_thread_number);

	lcp_solver_type = ChSystem::LCP_DEM;

	LCP_solver_speed = new ChLcpSolverDEM();

	LCP_solver_stab = new ChLcpSolverDEM();

	collision_system = new ChCollisionSystemBullet(max_objects, scene_size);

	contact_container = new ChContactContainerDEM;
}


void ChSystemDEM::SetLcpSolverType(eCh_lcpSolver mval)
{
	if (mval != LCP_DEM)
		return;

	lcp_solver_type = LCP_DEM;

	if (LCP_solver_speed)
		delete LCP_solver_speed;
	LCP_solver_speed = new ChLcpSolverDEM();

	if (LCP_solver_stab)
		delete LCP_solver_stab;
	LCP_solver_stab = new ChLcpSolverDEM();

	if (LCP_descriptor)
		delete LCP_descriptor;
	LCP_descriptor = new ChLcpSystemDescriptor;
	LCP_descriptor->SetNumThreads(parallel_thread_number);

	if (contact_container)
		delete contact_container;
	contact_container = new ChContactContainerDEM;
}

void ChSystemDEM::ChangeLcpSolverSpeed(ChLcpSolver* newsolver)
{
	if (dynamic_cast<ChLcpSolverDEM*>(newsolver))
		ChSystem::ChangeLcpSolverSpeed(newsolver);
}

void ChSystemDEM::ChangeContactContainer(ChContactContainerBase* newcontainer)
{
	if (dynamic_cast<ChContactContainerDEM*>(newcontainer))
		ChSystem::ChangeContactContainer(newcontainer);
}


}  // END_OF_NAMESPACE____