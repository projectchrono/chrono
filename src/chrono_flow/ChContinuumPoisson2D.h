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

#ifndef CHCONTINUUMPOISSON2D_H
#define CHCONTINUUMPOISSON2D_H

#include "chrono/fea/ChContinuumMaterial.h"
#include "chrono_flow/ChFlowApi.h"

using namespace chrono::fea;

namespace chrono {
namespace flow {

/// @addtogroup chrono_fea
/// @{

/// Class for the basic properties of scalar fields P in 3D FEM problems
/// that can be described by Laplace PDEs of type
///    rho dP/dt + div [C] grad P = 0
class ChFlowApi ChContinuumPoisson2D : public ChContinuumMaterial {
  protected:
    ChMatrixDynamic<> ConstitutiveMatrix;  // constitutive matrix

  public:
    ChContinuumPoisson2D() { ConstitutiveMatrix.setIdentity(3, 3); }
    ChContinuumPoisson2D(const ChContinuumPoisson2D& other) : ChContinuumMaterial(other) {
        ConstitutiveMatrix = other.ConstitutiveMatrix;
    }
    virtual ~ChContinuumPoisson2D() {}

    /// Get the constitutive matrix [C] to compute the bilinear form in the weak formulation
    ChMatrixDynamic<>& GetConstitutiveMatrix() { return ConstitutiveMatrix; }

    /// Get the rho multiplier for the 'rho dP/dt term', if any (default, none)
    virtual double Get_DtMultiplier() { return 0; }
};

/// @} chrono_fea

}  // end namespace flow
}  // end namespace chrono

#endif
