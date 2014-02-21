//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChMaterialSurfaceDEM.cpp
//
// ------------------------------------------------
//             www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChMaterialSurfaceDEM.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyDEM.h"

namespace chrono
{


// Calculate composite material properties as a combination of the physical
// properties of the two specified materials.
ChCompositeMaterialDEM
ChMaterialSurfaceDEM::CompositeMaterial(const ChSharedPtr<ChMaterialSurfaceDEM>& mat1,
                                        const ChSharedPtr<ChMaterialSurfaceDEM>& mat2)
{
	ChCompositeMaterialDEM mat;

	float inv_E = (1 - mat1->poisson_ratio * mat1->poisson_ratio) / mat1->young_modulus
	            + (1 - mat2->poisson_ratio * mat2->poisson_ratio) / mat2->young_modulus;
	float inv_G = 2 * (2 + mat1->poisson_ratio) * (1 - mat1->poisson_ratio) / mat1->young_modulus
	            + 2 * (2 + mat2->poisson_ratio) * (1 - mat2->poisson_ratio) / mat2->young_modulus;
	
	mat.E_eff = 1 / inv_E;
	mat.G_eff = 1 / inv_G;

	mat.mu_eff    = std::min(mat1->static_friction, mat2->static_friction);

	mat.cr_eff    = (mat1->restitution + mat2->restitution) / 2;
	mat.alpha_eff = (mat1->dissipation_factor + mat2->dissipation_factor) / 2;

	return mat;
}


} // END_OF_NAMESPACE____


// eof
