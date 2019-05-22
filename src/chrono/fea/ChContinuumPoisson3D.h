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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHCONTINUUMPOISSON3D_H
#define CHCONTINUUMPOISSON3D_H

#include "chrono/physics/ChContinuumMaterial.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

/// Class for the basic properties of scalar fields P in 3D FEM problems
/// that can be described by Laplace PDEs of type
///    rho dP/dt + div [C] grad P = 0
class ChApi ChContinuumPoisson3D : public ChContinuumMaterial {
  protected:
    ChMatrixDynamic<> ConstitutiveMatrix;  // constitutive matrix

  public:
    ChContinuumPoisson3D() {
        ConstitutiveMatrix.Resize(3, 3);
        ConstitutiveMatrix.SetIdentity();
    }
    ChContinuumPoisson3D(const ChContinuumPoisson3D& other) : ChContinuumMaterial(other) {
        ConstitutiveMatrix.CopyFromMatrix(other.ConstitutiveMatrix);
    }
    virtual ~ChContinuumPoisson3D() {}

    /// Get the constitutive matrix [C] to compute the bilinear form in the weak formulation
    ChMatrixDynamic<>& Get_ConstitutiveMatrix() { return ConstitutiveMatrix; }

    /// Get the rho multiplier for the 'rho dP/dt term', if any (default, none)
    virtual double Get_DtMultiplier() { return 0; }
};

/// @} chrono_fea

}  // end namespace fea
}  // end namespace chrono

#endif
