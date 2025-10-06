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
// Authors: Alessandro Tasora 
// =============================================================================

#ifndef CHMATERIALPOISSON_H
#define CHMATERIALPOISSON_H

#include "chrono/fea/ChMaterial3DDensity.h"
#include "chrono/core/ChMatrix.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{


/// Class for the basic properties of scalar fields P in 3D FEM problems
/// that can be described by PDEs of type
///    rho dP/dt + div [C] grad P = 0
class ChApi ChMaterialPoisson : public ChMaterial3DDensity {
public:
    ChMaterialPoisson() { constitutiveMatrix.setIdentity(3, 3); }

    virtual ~ChMaterialPoisson() {}

    /// The constitutive matrix [C] to compute the bilinear form in the weak formulation.
    /// You can modify its values, for setting material property.
    ChMatrixDynamic<>& ConstitutiveMatrix() { return constitutiveMatrix; }

    /// Get the rho multiplier for the 'rho dP/dt term', if any (default, none)
    virtual double Get_DtMultiplier() { return 0; }

protected:
    ChMatrixDynamic<> constitutiveMatrix;  // constitutive matrix
};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
