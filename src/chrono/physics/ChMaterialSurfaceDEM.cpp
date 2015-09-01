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

#include <cmath>
#include <algorithm>

#include "physics/ChMaterialSurfaceDEM.h"
#include "physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChMaterialSurfaceDEM> a_registration_ChMaterialSurfaceDEM;


// Constructors for a DEM material
ChMaterialSurfaceDEM::ChMaterialSurfaceDEM()
    : young_modulus(2e5),
      poisson_ratio(0.3f),
      static_friction(0.6f),
      sliding_friction(0.6f),
      restitution(0.4f),
      cohesion(0),
      kn(2e5),
      kt(2e5),
      gn(40),
      gt(20) {
}

ChMaterialSurfaceDEM::ChMaterialSurfaceDEM(const ChMaterialSurfaceDEM& other) {
    young_modulus = other.young_modulus;
    poisson_ratio = other.poisson_ratio;
    static_friction = other.static_friction;
    sliding_friction = other.sliding_friction;
    restitution = other.restitution;
    cohesion = other.cohesion;
    kn = other.kn;
    kt = other.kt;
    gn = other.gn;
    gt = other.gt;
}

// Calculate composite material properties as a combination of the physical
// properties of the two specified materials.
ChCompositeMaterialDEM ChMaterialSurfaceDEM::CompositeMaterial(const ChSharedPtr<ChMaterialSurfaceDEM>& mat1,
                                                               const ChSharedPtr<ChMaterialSurfaceDEM>& mat2) {
    ChCompositeMaterialDEM mat;

    float inv_E = (1 - mat1->poisson_ratio * mat1->poisson_ratio) / mat1->young_modulus +
                  (1 - mat2->poisson_ratio * mat2->poisson_ratio) / mat2->young_modulus;
    float inv_G = 2 * (2 + mat1->poisson_ratio) * (1 - mat1->poisson_ratio) / mat1->young_modulus +
                  2 * (2 + mat2->poisson_ratio) * (1 - mat2->poisson_ratio) / mat2->young_modulus;

    mat.E_eff = 1 / inv_E;
    mat.G_eff = 1 / inv_G;

    mat.mu_eff = std::min<float>(mat1->static_friction, mat2->static_friction);

    mat.cr_eff = (mat1->restitution + mat2->restitution) / 2;

    mat.cohesion_eff = std::min<float>(mat1->cohesion, mat2->cohesion);

    mat.kn = (mat1->kn + mat2->kn) / 2;
    mat.kt = (mat1->kt + mat2->kt) / 2;
    mat.gn = (mat1->gn + mat2->gn) / 2;
    mat.gt = (mat1->gt + mat2->gt) / 2;

    return mat;
}

}  // end namespace chrono
