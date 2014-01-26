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
#include "collision/ChCModelBulletBody.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;


ChContactDEM::ChContactDEM(collision::ChModelBulletBody*     mod1,
                           collision::ChModelBulletBody*     mod2,
                           const collision::ChCollisionInfo& cinfo)
{
	Reset(mod1, mod2, cinfo);
}


void
ChContactDEM::Reset(collision::ChModelBulletBody*     mod1,
                    collision::ChModelBulletBody*     mod2,
                    const collision::ChCollisionInfo& cinfo)
{
	assert(cinfo.distance < 0);

	m_mod1 = mod1;
	m_mod2 = mod2;

	ChBodyDEM* body1 = (ChBodyDEM*) m_mod1->GetBody();
	ChBodyDEM* body2 = (ChBodyDEM*) m_mod2->GetBody();

	// Calculate and store kinematic data for this contact
	// TODO: this should really be in cinfo...
	m_kdata.p1 = cinfo.vpA;
	m_kdata.p2 = cinfo.vpB;
	m_kdata.delta = cinfo.distance;
	m_kdata.normal = cinfo.vN;

	// Contact plane
	ChVector<> Vx, Vy, Vz;
	cinfo.vN.DirToDxDyDz(Vx, Vy, Vz);
	m_kdata.contact_plane.Set_A_axis(Vx, Vy, Vz);

	m_kdata.p1_loc = body1->Point_World2Body(m_kdata.p1);
	m_kdata.p2_loc = body2->Point_World2Body(m_kdata.p2);
	m_kdata.relvel = body2->RelPoint_AbsSpeed(m_kdata.p2_loc)
	               - body1->RelPoint_AbsSpeed(m_kdata.p1_loc);
	m_kdata.relvel_n = m_kdata.relvel.Dot(m_kdata.normal) * m_kdata.normal;
	m_kdata.relvel_t = m_kdata.relvel - m_kdata.relvel_n;

	// Calculate contact force
	ChMaterialSurfaceDEM::CalculateForce(body1, body2, m_kdata, m_force);
}


void
ChContactDEM::ConstraintsFbLoadForces(double factor)
{
	ChBodyDEM* body1 = (ChBodyDEM*) m_mod1->GetBody();
	ChBodyDEM* body2 = (ChBodyDEM*) m_mod2->GetBody();

	ChVector<> force1_loc = body1->Dir_World2Body(m_force);
	ChVector<> force2_loc = body2->Dir_World2Body(m_force);
	ChVector<> torque1_loc = Vcross(m_kdata.p1_loc,  force1_loc);
	ChVector<> torque2_loc = Vcross(m_kdata.p2_loc, -force2_loc);

	body1->Variables().Get_fb().PasteSumVector(m_force*factor,    0,0);
	body1->Variables().Get_fb().PasteSumVector(torque1_loc*factor,3,0);

	body2->Variables().Get_fb().PasteSumVector(-m_force*factor,   0,0);
	body2->Variables().Get_fb().PasteSumVector(torque2_loc*factor,3,0);
}


ChCoordsys<>
ChContactDEM::GetContactCoords()
{
	ChCoordsys<> mcsys;
	ChQuaternion<float> mrot = m_kdata.contact_plane.Get_A_quaternion();

	mcsys.rot.Set(mrot.e0, mrot.e1, mrot.e2, mrot.e3);
	mcsys.pos = m_kdata.p2;

	return mcsys;
}


} // END_OF_NAMESPACE____


