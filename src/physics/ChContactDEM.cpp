//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChContactDEM.cpp
//
// ------------------------------------------------
//             www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChContactDEM.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyDEM.h"
#include "lcp/ChLcpConstraintTwoContactN.h"
#include "collision/ChCModelBulletDEM.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;


ChContactDEM::ChContactDEM(collision::ChModelBulletDEM*      mod1,
                           collision::ChModelBulletDEM*      mod2,
                           const collision::ChCollisionInfo& cinfo)
{
	Reset(mod1, mod2, cinfo);
}


void ChContactDEM::Reset(collision::ChModelBulletDEM*      mod1,
                         collision::ChModelBulletDEM*      mod2,
                         const collision::ChCollisionInfo& cinfo)
{
	m_mod1 = mod1;
	m_mod2 = mod2;

	m_p1 = cinfo.vpA;
	m_p2 = cinfo.vpB;
	m_delta = cinfo.distance;
	m_normal = cinfo.vN;

	assert(m_delta < 0);

	// Contact plane
	ChVector<> Vx, Vy, Vz;
	cinfo.vN.DirToDxDyDz(Vx, Vy, Vz);
	m_contact_plane.Set_A_axis(Vx, Vy, Vz);

	ChBodyDEM* body1 = m_mod1->GetBody();
	ChBodyDEM* body2 = m_mod2->GetBody();

	// Calculate composite material properties
	double kn, gn, kt, mu;
	ChMaterialSurfaceDEM::compositeMaterial(body1->GetMaterialSurfaceDEM(),
	                                        body2->GetMaterialSurfaceDEM(),
	                                        kn, gn, kt, mu);

	// Initialize contact force to zero
	m_force = VNULL;

	// Normal spring force
	m_force -= kn * pow(-m_delta, 1.5) * cinfo.vN;

	// Normal damping force
	ChVector<> p1_loc = body1->Point_World2Body(m_p1);
	ChVector<> p2_loc = body2->Point_World2Body(m_p2);
	ChVector<> relvel = (body2->RelPoint_AbsSpeed(p2_loc))-(body1->RelPoint_AbsSpeed(p1_loc));
	ChVector<> relvel_n = relvel.Dot(cinfo.vN) * cinfo.vN;
	m_force += gn * pow(-m_delta, 0.5) * relvel_n;

	// Tangential force
	ChVector<> relvel_t = relvel - relvel_n;
	double     relvel_t_mag = relvel_t.Length();

	if (relvel_t_mag > 1e-4) {
		double dT = body1->GetSystem()->GetStep();
		double slip = relvel_t_mag * dT;
		double force_n_mag = m_force.Length();
		double force_t_mag = kt * slip;

		if (force_t_mag < mu * force_n_mag)
			force_t_mag = mu * force_n_mag;

		m_force += (force_t_mag / relvel_t_mag) * relvel_t;
	}
}


void ChContactDEM::ConstraintsFbLoadForces(double factor)
{
	ChBodyDEM* body1 = m_mod1->GetBody();
	ChBodyDEM* body2 = m_mod2->GetBody();

	ChVector<> p1_loc = body1->Point_World2Body(m_p1);
	ChVector<> p2_loc = body2->Point_World2Body(m_p2);
	ChVector<> force1_loc = body1->Dir_World2Body(m_force);
	ChVector<> force2_loc = body2->Dir_World2Body(m_force);
	ChVector<> torque1_loc = Vcross(p1_loc,  force1_loc);
	ChVector<> torque2_loc = Vcross(p2_loc, -force2_loc);

	body1->Variables().Get_fb().PasteSumVector(m_force*factor,    0,0);
	body1->Variables().Get_fb().PasteSumVector(torque1_loc*factor,3,0);

	body2->Variables().Get_fb().PasteSumVector(-m_force*factor,   0,0);
	body2->Variables().Get_fb().PasteSumVector(torque2_loc*factor,3,0);
}


ChCoordsys<> ChContactDEM::GetContactCoords()
{
	ChCoordsys<> mcsys;
	ChQuaternion<float> mrot = m_contact_plane.Get_A_quaternion();
	mcsys.rot.Set(mrot.e0, mrot.e1, mrot.e2, mrot.e3);
	mcsys.pos = this->m_p2;
	return mcsys;
}


} // END_OF_NAMESPACE____


