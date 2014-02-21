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

// Memory leak debugger (must be included last).
#include "core/ChMemory.h"


namespace chrono
{

using namespace collision;
using namespace geometry;


// Initialize static members
ChContactDEM::NormalForceModel     ChContactDEM::m_normalForceModel     = ChContactDEM::HuntCrossley;
ChContactDEM::TangentialForceModel ChContactDEM::m_tangentialForceModel = ChContactDEM::SimpleCoulombSliding;

double ChContactDEM::m_minSlipVelocity = 1e-4;


// Construct a new DEM contact between two models using the specified contact pair information.
ChContactDEM::ChContactDEM(collision::ChModelBulletBody*     mod1,
                           collision::ChModelBulletBody*     mod2,
                           const collision::ChCollisionInfo& cinfo)
{
	Reset(mod1, mod2, cinfo);
}


// Worker function for populating a new or reusing an exsting contact.
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

	// Contact points, penetration, normal
	m_p1 = cinfo.vpA;
	m_p2 = cinfo.vpB;
	m_delta = -cinfo.distance;
	m_normal = cinfo.vN;

	// Contact plane
	ChVector<> Vx, Vy, Vz;
	cinfo.vN.DirToDxDyDz(Vx, Vy, Vz);
	m_contact_plane.Set_A_axis(Vx, Vy, Vz);

	// Contact points in local frames
	m_p1_loc = body1->Point_World2Body(m_p1);
	m_p2_loc = body2->Point_World2Body(m_p2);

	// Calculate contact force
	CalculateForce();
}


// Calculate the contact force to be applied to body2, based on the
// globally specified contact force models (normal and tangential).
void
ChContactDEM::CalculateForce()
{
	ChBodyDEM* body1 = (ChBodyDEM*) m_mod1->GetBody();
	ChBodyDEM* body2 = (ChBodyDEM*) m_mod2->GetBody();

	double dT = body1->GetSystem()->GetStep();

	// Relative velocity at contact
	ChVector<> vel2         = body2->PointSpeedLocalToParent(m_p2_loc);
	ChVector<> vel1         = body1->PointSpeedLocalToParent(m_p1_loc);
	ChVector<> relvel       = vel2 - vel1;
	double     relvel_n_mag = relvel.Dot(m_normal);
	ChVector<> relvel_n     = relvel_n_mag * m_normal;
	ChVector<> relvel_t     = relvel - relvel_n;
	double     relvel_t_mag = relvel_t.Length();

	// Calculate effective mass
	double m_eff = body1->GetMass() * body2->GetMass() / (body1->GetMass() + body2->GetMass());

	// Calculate effective contact radius
	//// TODO:  how can I get this with current collision system!?!?!?
	double R_eff = 1;

	// Calculate composite material properties
	ChCompositeMaterialDEM mat = ChMaterialSurfaceDEM::CompositeMaterial(body1->GetMaterialSurfaceDEM(), body2->GetMaterialSurfaceDEM());

	// Normal force
	double forceN;

	switch (m_normalForceModel) {
	case HuntCrossley:
		{
		double kn = (4.0 / 3) * mat.E_eff * std::sqrt(R_eff);
		double forceN_elastic = kn * m_delta * std::sqrt(m_delta);
		double forceN_dissipation = 1.5f * mat.alpha_eff * forceN_elastic * relvel_n_mag;
		forceN = forceN_elastic - forceN_dissipation;
		}
		break;
	}

	m_force = forceN * m_normal;

	// Tangential force
	if (relvel_t_mag <= m_minSlipVelocity)
		return;

	double forceT;

	switch (m_tangentialForceModel) {
	case SimpleCoulombSliding:
		forceT = mat.mu_eff * std::abs(forceN);
		break;
	case LinearSpring:
		{
		double kt = 2e7;
		double slip = relvel_t_mag * dT;
		forceT = std::min(kt * slip, mat.mu_eff * std::abs(forceN));
		}
		break;
	case LinearDampedSpring:
		////double slip = relvel_t_mag * dT;
		////double kt = 8 * mat.G_eff * std::sqrt(R_eff);
		////double forceT_elastic = kt * m_delta * std::sqrt(slip);
		////double forceT_dissipation = 0;
		////forceT = std::min(forceT_elastic + forceT_dissipation, mat.mu_eff * std::abs(forceN));

		break;
	}

	m_force -= (forceT / relvel_t_mag) * relvel_t;
}


// Include the contact force from this contact into the body forces and
// torques of the two bodies involved in this contact.  Recall that the
// contact force as calculated must be applied to body2 and inverted for
// body1.
void
ChContactDEM::ConstraintsFbLoadForces(double factor)
{
	ChBodyDEM* body1 = (ChBodyDEM*) m_mod1->GetBody();
	ChBodyDEM* body2 = (ChBodyDEM*) m_mod2->GetBody();

	ChVector<> force1_loc = body1->Dir_World2Body(m_force);
	ChVector<> force2_loc = body2->Dir_World2Body(m_force);
	ChVector<> torque1_loc = Vcross(m_p1_loc, -force1_loc);
	ChVector<> torque2_loc = Vcross(m_p2_loc,  force2_loc);

	body1->Variables().Get_fb().PasteSumVector(-m_force*factor,    0,0);
	body1->Variables().Get_fb().PasteSumVector(torque1_loc*factor,3,0);

	body2->Variables().Get_fb().PasteSumVector(m_force*factor,   0,0);
	body2->Variables().Get_fb().PasteSumVector(torque2_loc*factor,3,0);
}


// Return the coordinate system for this contact (centered at point P2
// and oriented based on the contact plane and normal)
ChCoordsys<>
ChContactDEM::GetContactCoords() const
{
	ChCoordsys<> mcsys;
	ChQuaternion<float> mrot = m_contact_plane.Get_A_quaternion();

	mcsys.rot.Set(mrot.e0, mrot.e1, mrot.e2, mrot.e3);
	mcsys.pos = m_p2;

	return mcsys;
}


} // END_OF_NAMESPACE____


