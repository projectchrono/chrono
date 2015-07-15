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

#include "physics/ChSystemDEM.h"
#include "physics/ChContactContainerDEM.h"
#include "physics/ChContactContainerDEM__old.h"

#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpSolverDEM.h"

#include "collision/ChCCollisionSystemBullet.h"

namespace chrono {

ChClassRegister<ChSystemDEM> a_registration_ChSystemDEM;

ChSystemDEM::ChSystemDEM(bool use_material_properties, bool use_history, unsigned int max_objects, double scene_size)
    : ChSystem(max_objects, scene_size, false),
      m_use_mat_props(use_material_properties),
      m_use_history(use_history),
      m_contact_model(ContactForceModel::Hertz) {
    LCP_descriptor = new ChLcpSystemDescriptor;
    LCP_descriptor->SetNumThreads(parallel_thread_number);

    lcp_solver_type = ChSystem::LCP_DEM;

    LCP_solver_speed = new ChLcpSolverDEM();

    LCP_solver_stab = new ChLcpSolverDEM();

    collision_system = new collision::ChCollisionSystemBullet(max_objects, scene_size);

    contact_container = new ChContactContainerDEM;
    contact_container->SetSystem(this);

    m_minSlipVelocity = 1e-4; 
    m_characteristicVelocity = 1; 
}

void ChSystemDEM::SetLcpSolverType(eCh_lcpSolver mval) {

    ChSystem::SetLcpSolverType(mval);

    if (contact_container)
        delete contact_container;
    contact_container = new ChContactContainerDEM;
    contact_container->SetSystem(this);
}

/*
void ChSystemDEM::ChangeLcpSolverSpeed(ChLcpSolver* newsolver) {
    if (dynamic_cast<ChLcpSolverDEM*>(newsolver))
        ChSystem::ChangeLcpSolverSpeed(newsolver);
}
*/

void ChSystemDEM::ChangeContactContainer(ChContactContainerBase* newcontainer) {
    if (dynamic_cast<ChContactContainerDEM*>(newcontainer))
        ChSystem::ChangeContactContainer(newcontainer);
}

}  // end namespace chrono
