//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChFunction.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChMaterialSurfaceDEM.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyDEM.h"

namespace chrono
{

// Initialize static members
ChMaterialSurfaceDEM::NormalContactModel     ChMaterialSurfaceDEM::m_normalContactModel     = ChMaterialSurfaceDEM::DefaultNormal;
ChMaterialSurfaceDEM::TangentialContactModel ChMaterialSurfaceDEM::m_tangentialContactModel = ChMaterialSurfaceDEM::DefaultTangential;


// Default normal contact force model
//
void
NormalForce_Default(const ChSharedPtr<ChMaterialSurfaceDEM>& mat1,
                    const ChSharedPtr<ChMaterialSurfaceDEM>& mat2,
                    const ChContactKinematicsDEM&            kdata,
                    ChVector<>&                              force)
{
	// Calculate composite material properties
	double kn = (mat1->GetNormalStiffness() + mat1->GetNormalStiffness()) / 2;
	double gn = (mat1->GetNormalDamping()   + mat1->GetNormalDamping()  ) / 2;

	// Normal spring force
	force -= kn * pow(-kdata.delta, 1.5) * kdata.normal;

	// Normal damping force
	force += gn * pow(-kdata.delta, 0.5) * kdata.relvel_n;
}


// Machado-Flores normal force model
//
void
NormalForce_Flores(const ChSharedPtr<ChMaterialSurfaceDEM>& mat1,
                   const ChSharedPtr<ChMaterialSurfaceDEM>& mat2,
                   const ChContactKinematicsDEM&            kdata,
                   ChVector<>&                              force)
{
}


// Default tangential force model
//
void
TangentialForce_Default(const ChSharedPtr<ChMaterialSurfaceDEM>& mat1,
                        const ChSharedPtr<ChMaterialSurfaceDEM>& mat2,
                        const ChContactKinematicsDEM&            kdata,
                        double                                   dT,
                        ChVector<>&                              force)
{
	double  relvel_t_mag = kdata.relvel_t.Length();

	if (relvel_t_mag > 1e-4) {
		// Calculate composite material properties
		double kt = (mat1->GetTangentialStiffness() + mat1->GetTangentialStiffness()) / 2;
		double mu = (mat1->GetSfriction()           + mat1->GetSfriction()          ) / 2;

		// Calculate magnitude of tangential force
		double slip = relvel_t_mag * dT;
		double force_n_mag = force.Length();
		double force_t_mag = kt * slip;

		// Apply Coulomb friction law
		if (force_t_mag < mu * force_n_mag)
			force_t_mag = mu * force_n_mag;

		force += (force_t_mag / relvel_t_mag) * kdata.relvel_t;
	}
}


//
//
void
ChMaterialSurfaceDEM::CalculateForce(ChBodyDEM*                    body1,
                                     ChBodyDEM*                    body2,
                                     const ChContactKinematicsDEM& kdata,
                                     ChVector<>&                   force)
{
	// Initialize contact force to zero
	force = VNULL;

	// Include normal contact forces
	switch (m_normalContactModel) {
	case Flores:
		NormalForce_Flores(body1->GetMaterialSurfaceDEM(), body2->GetMaterialSurfaceDEM(), kdata, force);
		break;
	default:
		NormalForce_Default(body1->GetMaterialSurfaceDEM(), body2->GetMaterialSurfaceDEM(), kdata, force);
		break;
	}

	// Include tangential contact forces
	switch (m_tangentialContactModel) {
	default:
		TangentialForce_Default(body1->GetMaterialSurfaceDEM(), body2->GetMaterialSurfaceDEM(), kdata, body1->GetSystem()->GetStep(), force);
		break;
	}
}



} // END_OF_NAMESPACE____


// eof
